// Copyright (C) 2025-2026 356C LLC
// SPDX-License-Identifier: GPL-3.0-or-later
#if HELIX_HAS_ACE

/**
 * @file ams_backend_ace.cpp
 * @brief ACE (AnyCubic ACE Pro) backend implementation
 *
 * Primary path: WebSocket subscription to ace Klipper object (ValgACE).
 * Fallback path: REST polling via /server/ace/ endpoints (BunnyACE/DuckACE).
 * See ams_backend_ace.h for full documentation.
 */

#include "ams_backend_ace.h"

#include "ui_insert_notice.h"
#include "ui_toast_manager.h"
#include "ui_update_queue.h"

#include "ams_bypass_policy.h"
#include "ams_device_action_value.h"
#include "ams_device_sections.h"
#include "ams_tool_map_sync.h"
#include "exception_policy.h"
#include "helix_thread.h"
#include "i_moonraker_api.h"
#include "i_moonraker_client.h"
#include "json_utils.h"
#include "lane_apply.h"
#include "lane_legacy_migration.h"
#include "lane_source_store.h"
#include "lane_translation.h"
#include "lvgl/src/others/translation/lv_translation.h"
#include "macro_choice.h"
#include "post_op_cooldown_manager.h"
#include "printer_discovery.h"
#include "settings_manager.h"
#include "spdlog/spdlog.h"
#include "text_io.h"

#include <spdlog/fmt/fmt.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <map>
#include <tuple>

namespace helix {

using json = nlohmann::json;

/// Max consecutive /server/ace/info failures before giving up on REST fallback
static constexpr int kMaxInfoFetchFailures = 3;

namespace {

/// The model a fork unit's `protocol` names, empty when it names none known.
std::string ace_model_for_protocol(const std::string& protocol) {
    if (protocol == "ace1_json") {
        return "ACEPRO";
    }
    if (protocol == "ace2_proto") {
        return "ACEPRO2";
    }
    return {};
}

/// Whether a fork unit's `connection_state` says it is connected; nullopt for
/// a state this backend does not know, which leaves the unit as it was.
std::optional<bool> ace_link_from_state(const std::string& state) {
    if (state == "connected") {
        return true;
    }
    if (state == "initializing" || state == "connecting" || state == "reconnecting" ||
        state == "disabled") {
        return false;
    }
    return std::nullopt;
}

} // namespace

// ============================================================================
// Construction / Destruction
// ============================================================================

nlohmann::json AmsBackendAce::required_status_objects(const helix::PrinterDiscovery& hw) {
    nlohmann::json objects = nlohmann::json::object();
    if (hw.mmu_type() != AmsType::ACE) {
        return objects;
    }
    // Subscribe the real detected object name(s) so slot colors, materials, status
    // and dryer state push live via get_status().
    if (hw.ace_object_names().empty()) {
        objects["ace"] = nullptr;
    } else {
        for (const auto& name : hw.ace_object_names()) {
            objects[name] = nullptr;
        }
    }
    return objects;
}

AmsBackendAce::AmsBackendAce(IMoonrakerAPI* api, IMoonrakerClient* client)
    : AmsSubscriptionBackend(api, client) {
    // Initialize system info with ACE defaults
    system_info_.type = AmsType::ACE;
    system_info_.type_name = "ACE";
    system_info_.supports_bypass = false;
}

AmsBackendAce::~AmsBackendAce() {
    stop_rest_fallback();
}

// ============================================================================
// AmsSubscriptionBackend Hooks
// ============================================================================

void AmsBackendAce::on_started() {
    spdlog::info("[ACE] Backend started — querying initial filament_hub/ace state via WebSocket");

    // A restart re-subscribes and re-parses from a fresh initial query, so
    // the insert edge re-baselines too: a prev status carried across a
    // disconnect would judge the first post-reconnect frame as an insert,
    // and an evidence map carried across a printer swap would compare
    // spools that never shared a bay.
    {
        std::lock_guard<std::mutex> lock(mutex_);
        user_temps_.clear();
        prev_slot_status_.clear();
        park_states_.clear();
        last_spool_evidence_.clear();
        pending_insert_reads_.clear();
        unit_actions_.clear();
        slot_tools_.clear();
        unit_links_.clear();
        // The load's ack died with the old session, so nothing would end it.
        if (own_load_ && (system_info_.action == AmsAction::LOADING ||
                          system_info_.action == AmsAction::UNLOADING)) {
            system_info_.action = AmsAction::IDLE;
        }
        own_load_ = false;
    }

    // Load persisted per-slot overrides from the shared FilamentSlotOverrideStore
    // BEFORE issuing the initial status query — otherwise the first status
    // callback (libhv background thread) could fire and parse slots before
    // overrides_ is populated, so the first EVENT_STATE_CHANGED frame would
    // miss override data. load_blocking runs on this (main) thread; the
    // Moonraker DB callback fires on the libhv event loop, so the two threads
    // don't interfere. Migration from helix-screen:ace_slot_overrides to
    // lane_data happens automatically inside load_blocking the first time
    // lane_data is empty (Task 8).
    if (api_) {
        auto loaded =
            helix::ams::make_loaded_override_store(api_, "ace", get_type(), backend_log_tag(),
                                                   "lane_data", /*migrates_legacy_namespace=*/true);
        if (loaded.store) {
            helix::ams::ingest_legacy_records(*loaded.store, helix::ams::LegacyLockKeys::LaneData,
                                              backend_index());
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            override_store_ = std::move(loaded.store);
            overrides_ = std::move(loaded.overrides);
        }
    }

    // start() refuses to run without a client, but a directly-constructed
    // backend (tests) reaches here with none and the query below would
    // dereference null (#1650).
    if (!client_) {
        spdlog::debug("[ACE] No Moonraker client — skipping initial status query");
        return;
    }

    auto token = op_lifetime_.token();

    // Query all known Klipper object names directly (works if driver has
    // get_status()). Native Anycubic GoKlipper registers the object as
    // `filament_hub`; community ValgACE/BunnyACE/DuckACE register it as `ace`;
    // the Kobra S1 mainline-Python fork registers each unit as
    // `ace_instance_N` (#1107), as many as discovery found: a unit left out
    // of this query has no slots until its spools next change.
    // printer.objects.query tolerates unknown objects (absent from the
    // result) and select_ace_object handles absent keys, so over-querying is
    // safe.
    json objects_to_query = json::object();
    objects_to_query["filament_hub"] = nullptr;
    objects_to_query["ace"] = nullptr;
    std::vector<std::string> instances;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        instances = discovered_instances_;
    }
    if (instances.empty()) {
        // No discovery to go by: the first four, as the subscription falls
        // back to `ace` without one.
        for (int i = 0; i < 4; ++i) {
            instances.push_back(fmt::format("ace_instance_{}", i));
        }
    }
    for (const auto& name : instances) {
        objects_to_query[name] = nullptr;
    }

    json params = {{"objects", objects_to_query}};

    client_->send_jsonrpc(
        "printer.objects.query", params,
        [this, token](const json& response) {
            // L081 Mechanism C: defer all member access (parse_ace_object,
            // info_fetched_, emit_event, start_rest_fallback) to main thread.
            // `this` capture is safe because the defer body checks the token
            // and skips if the owner has been destroyed.
            token.defer("AmsBackendAce::on_started_query", [this, response]() {
                // Prefer the native `filament_hub` key; fall back to the
                // community `ace` key. Commit to the subscription path ONLY
                // when the matched object actually carries slot data — a
                // manager-only object (Kobra S1 fork: ace_instances/current_index,
                // no slots) must fall through to the REST bridge (#1069).
                const json* ace_data = nullptr;
                std::string matched_key;
                if (response.contains("result") && response["result"].contains("status")) {
                    ace_data = select_ace_object(response["result"]["status"], &matched_key);
                }

                if (ace_data) {
                    // Native/ValgACE path: got real data from get_status()
                    spdlog::info("[ACE] Klipper '{}' object has status data — "
                                 "using native WebSocket subscription",
                                 matched_key);
                    if (matched_key.rfind("ace_instance", 0) == 0) {
                        // The fork: every instance is a unit of its own.
                        {
                            std::lock_guard<std::mutex> lock(mutex_);
                            instance_objects_ = true;
                        }
                        for (const auto& [unit, instance] :
                             ace_instance_objects(response["result"]["status"])) {
                            parse_ace_object(*instance, unit);
                        }
                    } else {
                        parse_ace_object(*ace_data);
                    }
                    // A manager-shaped `ace` can ride the same response and
                    // states the seat (current_index); parse it AFTER the
                    // slot-bearing object so the seat stamps onto populated
                    // slots (#1069).
                    if (const json* manager = manager_ace_object(response["result"]["status"])) {
                        parse_ace_object(*manager);
                    }
                    info_fetched_.store(true);
                    emit_event(EVENT_STATE_CHANGED);

                } else {
                    // BunnyACE/DuckACE path: no get_status(), try REST fallback
                    spdlog::info("[ACE] Klipper filament_hub/ace object has no status data — "
                                 "trying REST bridge fallback (/server/ace/*)");
                    start_rest_fallback();
                }
            });
        },
        [this, token](const MoonrakerError& err) {
            token.defer("AmsBackendAce::on_started_query_err", [this, err]() {
                spdlog::warn("[ACE] Initial filament_hub/ace query failed: {} — "
                             "trying REST fallback",
                             err.message);
                start_rest_fallback();
            });
        });
}

void AmsBackendAce::on_stopping() {
    // on_stopping() is called with mutex_ held — do NOT lock mutex_ here.
    // stop_rest_fallback uses its own rest_stop_mutex_, which is safe.
    stop_rest_fallback();
    op_lifetime_.invalidate();
}

void AmsBackendAce::handle_status(const json& status_obj) {
    if (use_rest_fallback_)
        return; // Using REST polling, ignore subscriptions

    const json* status = &status_obj;

    // Native Anycubic GoKlipper publishes under `filament_hub`; community
    // ValgACE under `ace`; the Kobra S1 mainline-Python fork under
    // `ace_instance_N` with a manager-shaped `ace` beside it (#1069, #1107).
    //
    // A frame carrying `ace_instance_N` keys is the fork: each instance is
    // unit N, and a notify frame carries only the objects that changed, so it
    // holds whichever units changed. The manager is parsed after them, so the
    // seat (current_index) lands on slots that are already populated.
    //
    // Otherwise a slot-bearing object is parsed first; notify frames carry
    // only CHANGED fields, so a delta with no slots array at all falls back to
    // the first non-empty object (the manager is excluded there via `skip`).
    const json* manager = manager_ace_object(*status);
    const auto instances = ace_instance_objects(*status);
    const json* ace_data = nullptr;
    if (instances.empty()) {
        ace_data = select_ace_object(*status, nullptr, /*require_slots=*/true);
        if (!ace_data) {
            ace_data = select_ace_object(*status, nullptr, /*require_slots=*/false, manager);
        }
    }

    if (instances.empty() && !ace_data && !manager)
        return;

    if (!instances.empty()) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            instance_objects_ = true;
        }
        for (const auto& [unit, instance] : instances) {
            parse_ace_object(*instance, unit);
        }
    }
    if (ace_data) {
        parse_ace_object(*ace_data);
    }
    if (manager) {
        parse_ace_object(*manager);
    }
    emit_event(EVENT_STATE_CHANGED);
}

// ============================================================================
// State Queries
// ============================================================================

AmsSystemInfo AmsBackendAce::get_system_info() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return system_info_;
}

// ============================================================================
// Path Visualization
// ============================================================================

PathTopology AmsBackendAce::get_topology() const {
    return PathTopology::HUB;
}

bool AmsBackendAce::seat_at_nozzle_locked() const {
    // Caller holds mutex_. A seated tool outranks the sensors: both are still
    // made with filament in the nozzle. Not while the driver swaps it out (the
    // seat stands until the new tool is in), and not while a feed it is
    // retrying keeps target_index on the seated tool short of the toolhead
    // sensor. The sensor ends that, because the driver persists target_index
    // and can leave it set on a tool that is loaded.
    return system_info_.filament_loaded && !driver_action_ &&
           !(path_sensors_seen_ && target_index_ >= 0 && !toolhead_sensor_);
}

bool AmsBackendAce::path_empty_under_seat_locked() const {
    // Caller holds mutex_. A driver that paused on a failed feed keeps the
    // tool it was feeding as current, with nothing on the path to show for it
    // (prestonbrown/helixscreen#1678).
    return seat_loaded_ && !driver_action_ && path_sensors_seen_ && !rdm_sensor_ &&
           !toolhead_sensor_;
}

PathSegment AmsBackendAce::get_filament_segment() const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (seat_at_nozzle_locked()) {
        return PathSegment::NOZZLE;
    }

    // Nothing at the nozzle, so anything the sensors see is a strand in flight.
    if (path_sensors_seen_) {
        if (toolhead_sensor_) {
            return PathSegment::TOOLHEAD;
        }
        if (rdm_sensor_) {
            return PathSegment::OUTPUT;
        }
    }

    return PathSegment::NONE;
}

PathSegment AmsBackendAce::get_slot_filament_segment(int slot_index) const {
    std::lock_guard<std::mutex> lock(mutex_);

    const SlotInfo* found = slot_info_locked(slot_index);
    if (found == nullptr) {
        return PathSegment::NONE;
    }
    const auto& slot = *found;

    if (system_info_.current_slot == slot_index && seat_at_nozzle_locked()) {
        return PathSegment::NOZZLE;
    }

    if (slot.status == SlotStatus::AVAILABLE || slot.status == SlotStatus::LOADED) {
        if (slot_parked_at_hub_locked(slot_index)) {
            // The segment the hub sensor places a strand at in get_filament_segment.
            return PathSegment::OUTPUT;
        }
        return PathSegment::SPOOL;
    }

    return PathSegment::NONE;
}

AmsBackendAce::ParkState AmsBackendAce::park_state_from_json(const json& slot_json) {
    if (slot_json.contains("park_state") && slot_json["park_state"].is_string()) {
        const auto& text = slot_json["park_state"].get_ref<const std::string&>();
        if (text == "parked") {
            return ParkState::Parked;
        }
        if (text == "loaded") {
            return ParkState::Loaded;
        }
    }
    return ParkState::Unknown;
}

bool AmsBackendAce::note_park_state_locked(int index, const json& slot_json) {
    const ParkState state = park_state_from_json(slot_json);
    auto it = park_states_.find(index);
    const ParkState before = it == park_states_.end() ? ParkState::Unknown : it->second;
    if (state == ParkState::Unknown) {
        park_states_.erase(index);
    } else {
        park_states_[index] = state;
    }
    return state != before;
}

bool AmsBackendAce::slot_parked_at_hub_locked(int slot_index) const {
    // Caller holds mutex_. Anything the driver is doing, and the seated slot,
    // are placed by the action, the seat and the sensors, never by a remembered
    // park.
    if (driver_action_ || target_index_ >= 0 || system_info_.action != AmsAction::IDLE ||
        system_info_.current_slot == slot_index) {
        return false;
    }
    auto it = park_states_.find(slot_index);
    return it != park_states_.end() && it->second == ParkState::Parked;
}

PathSegment AmsBackendAce::infer_error_segment() const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (system_info_.action == AmsAction::ERROR) {
        return PathSegment::HUB;
    }

    return PathSegment::NONE;
}

// ============================================================================
// Filament Operations
// ============================================================================

AmsError AmsBackendAce::do_load_filament(int slot_index) {
    auto err = validate_slot_index(slot_index);
    if (!err.success()) {
        return err;
    }

    spdlog::info("[ACE] Loading filament from slot {}", slot_index);

    // Set action optimistically so sidebar detects the LOADING → IDLE transition.
    // The ACE Klipper module may not report "status": "loading" during gcode execution,
    // leaving the sidebar stuck waiting for a transition that never starts. With
    // another tool seated the driver retracts it first, so the op starts as that.
    int tool = -1;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        tool = tool_for_slot_locked(slot_index);
        if (tool < 0) {
            return AmsErrorHelper::invalid_parameter("ACE slot " + std::to_string(slot_index) +
                                                     " has no tool from the driver yet");
        }
        const int seated = system_info_.current_slot;
        system_info_.action = (seat_loaded_ && seated >= 0 && seated != slot_index)
                                  ? AmsAction::UNLOADING
                                  : AmsAction::LOADING;
        own_load_ = true;
    }
    emit_event(EVENT_STATE_CHANGED);

    std::string gcode = "ACE_CHANGE_TOOL TOOL=" + std::to_string(tool);
    auto token = op_lifetime_.token();

    return execute_gcode(
        gcode,
        [this, token, slot_index]() {
            // L081 Mechanism C: marshal member writes (system_info_) to main.
            token.defer("AmsBackendAce::load_done", [this, slot_index]() {
                spdlog::info("[ACE] Load slot {} gcode completed", slot_index);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    system_info_.action = AmsAction::IDLE;
                    own_load_ = false;

                    // Where firmware states the seat itself, the ack is only
                    // "the command was accepted" and the seat waits for the
                    // driver to confirm it moved.
                    if (!manager_states_seat_) {
                        seat_from_local_index_locked(slot_index);

                        // Same derivation the parse paths use, so the next
                        // status frame re-applies this stamp instead of
                        // erasing it.
                        apply_seated_slot_stamp_locked();
                    }
                }
                PostOpCooldownManager::instance().schedule();
                emit_event(EVENT_STATE_CHANGED);
            });
        },
        [this, token](const MoonrakerError&) {
            // The send failed, so the optimistic LOADING has to be unwound or
            // the sidebar waits on a transition that never starts.
            token.defer("AmsBackendAce::load_err", [this]() {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    system_info_.action = AmsAction::IDLE;
                    own_load_ = false;
                }
                emit_event(EVENT_STATE_CHANGED);
            });
        },
        /*silent=*/false);
}

AmsError AmsBackendAce::do_unload_filament(int /*slot_index*/) {
    spdlog::info("[ACE] Unloading filament");

    {
        std::lock_guard<std::mutex> lock(mutex_);
        system_info_.action = AmsAction::UNLOADING;
    }
    emit_event(EVENT_STATE_CHANGED);

    std::string gcode = "ACE_CHANGE_TOOL TOOL=-1";
    auto token = op_lifetime_.token();

    return execute_gcode(
        gcode,
        [this, token]() {
            // L081 Mechanism C: marshal member writes (system_info_) to main.
            token.defer("AmsBackendAce::unload_done", [this]() {
                spdlog::info("[ACE] Unload gcode completed");
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    system_info_.action = AmsAction::IDLE;

                    // Symmetric with the load path: a driver that states the
                    // seat also clears it, and an unload it declined would
                    // otherwise read as an empty toolhead.
                    if (!manager_states_seat_) {
                        seat_from_local_index_locked(-1);

                        // Releases the stamp back to the status the parse
                        // wrote, rather than assuming AVAILABLE for a slot
                        // firmware may have called EMPTY.
                        apply_seated_slot_stamp_locked();
                    }
                }
                PostOpCooldownManager::instance().schedule();
                emit_event(EVENT_STATE_CHANGED);
            });
        },
        [this, token](const MoonrakerError&) {
            // The send failed, so the optimistic UNLOADING has to be unwound or
            // the sidebar waits on a transition that never starts.
            token.defer("AmsBackendAce::unload_err", [this]() {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    system_info_.action = AmsAction::IDLE;
                }
                emit_event(EVENT_STATE_CHANGED);
            });
        },
        /*silent=*/false);
}

AmsError AmsBackendAce::do_select_slot(int slot_index) {
    return do_load_filament(slot_index);
}

AmsError AmsBackendAce::do_change_tool(int tool_number) {
    int slot = tool_number;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        slot = slot_for_tool_locked(tool_number);
    }
    return do_load_filament(slot);
}

// ============================================================================
// Recovery Operations
// ============================================================================

AmsError AmsBackendAce::recover() {
    spdlog::info("[ACE] Attempting recovery");
    return execute_gcode("ACE_RECOVER");
}

AmsError AmsBackendAce::reset() {
    spdlog::info("[ACE] Resetting");
    return execute_gcode("ACE_RESET");
}

void AmsBackendAce::on_klipper_restarted() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        target_index_ = -1;
        driver_action_ = false;
        swap_hub_cleared_ = false;
        system_info_.pending_target_slot = -1;
    }
    AmsSubscriptionBackend::on_klipper_restarted();
}

AmsError AmsBackendAce::cancel() {
    spdlog::info("[ACE] Cancelling operation");

    // Invalidate outstanding load/unload callbacks so they don't
    // overwrite state after cancel completes
    op_lifetime_.invalidate();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        system_info_.action = AmsAction::IDLE;
    }
    emit_event(EVENT_STATE_CHANGED);

    return execute_gcode("ACE_CHANGE_TOOL TOOL=-1");
}

// ============================================================================
// Configuration
// ============================================================================

namespace {

/// Parse passes an insert whose tag read has not landed stays pending before
/// the no-read fallback verdict runs. The REST poll brings every bay through
/// the parse each kPollIntervalMs, so this bounds the wait on that path; the
/// WebSocket path is change-driven, where a read that never lands can leave
/// the insert pending until the next frame of any kind arrives - harmless,
/// since a late verdict is still judged against the occupant that was in the
/// bay when the spool went in.
constexpr int kAcePendingReadParsePasses = 8;

/// Put @p info's filament fields on @p slot so get_slot_info returns them at
/// once: identity (the catalog product included, or the editor snaps back to a
/// different variant), multi-colour and temperatures.
void write_filament_fields(SlotInfo& slot, const SlotInfo& info) {
    slot.assign_filament_fields(info);
}

} // namespace

void AmsBackendAce::forget_user_temps_locked(int slot_index) {
    if (slot_index >= 0 && static_cast<size_t>(slot_index) < user_temps_.size()) {
        user_temps_[static_cast<size_t>(slot_index)] = false;
    }
}

void AmsBackendAce::apply_hub_temps_locked(int slot_index, SlotInfo& slot,
                                           const nlohmann::json& slot_json) const {
    // No lane record carries temperatures and the stored record keeps one for
    // every edit, the hub's own included, so only an edit that moved them this
    // session marks them the user's.
    if (slot_index >= 0 && static_cast<size_t>(slot_index) < user_temps_.size() &&
        user_temps_[static_cast<size_t>(slot_index)]) {
        return;
    }
    slot.nozzle_temp_min = helix::json_util::safe_int(slot_json, "temp_min", slot.nozzle_temp_min);
    slot.nozzle_temp_max = helix::json_util::safe_int(slot_json, "temp_max", slot.nozzle_temp_max);
}

void AmsBackendAce::note_tag_frame_locked(int slot_index, helix::ace::TagRead tag_read,
                                          const nlohmann::json& slot_json,
                                          const std::optional<std::string>& material,
                                          const std::optional<uint32_t>& color,
                                          std::optional<bool> present) {
    if (slot_index < 0) {
        return;
    }
    const auto idx = static_cast<size_t>(slot_index);
    if (tag_readings_.size() <= idx) {
        tag_readings_.resize(idx + 1);
    }
    // An emptied bay holds no spool to have a tag, and a finished read that
    // found none, or a frame stating the hub's memory instead of a read, ends
    // the last one. A read not finished yet leaves it standing.
    if ((present.has_value() && !*present) || tag_read == helix::ace::TagRead::NoTag ||
        tag_read == helix::ace::TagRead::Memory) {
        tag_readings_[idx].reset();
        return;
    }
    if (tag_read != helix::ace::TagRead::Identified) {
        return;
    }
    helix::ams::TagReading reading;
    reading.material = material;
    if (color && helix::ams::is_declarable_color(*color)) {
        reading.color_rgb = color;
    }
    const int temp_min = helix::json_util::safe_int(slot_json, "temp_min", 0);
    const int temp_max = helix::json_util::safe_int(slot_json, "temp_max", 0);
    if (temp_min > 0) {
        reading.nozzle_temp_min = temp_min;
    }
    if (temp_max > 0) {
        reading.nozzle_temp_max = temp_max;
    }
    if (reading.empty()) {
        tag_readings_[idx].reset();
    } else {
        tag_readings_[idx] = std::move(reading);
    }
}

std::optional<helix::ams::TagReading> AmsBackendAce::tag_reading(int slot_index) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (slot_index < 0 || static_cast<size_t>(slot_index) >= tag_readings_.size()) {
        return std::nullopt;
    }
    return tag_readings_[static_cast<size_t>(slot_index)];
}

AmsError AmsBackendAce::apply_user_edit(int slot_index, const SlotInfo& info,
                                        const helix::ams::Observation& declared) {
    {
        std::lock_guard<std::mutex> lock(mutex_);

        SlotInfo* slot = mutable_slot_locked(slot_index);
        if (!slot) {
            return AmsErrorHelper::invalid_slot(lane_noun(), slot_index, 0);
        }
        const bool temps_moved = info.nozzle_temp_min != slot->nozzle_temp_min ||
                                 info.nozzle_temp_max != slot->nozzle_temp_max;
        if (temps_moved) {
            const auto idx = static_cast<size_t>(slot_index);
            if (user_temps_.size() <= idx) {
                user_temps_.resize(idx + 1, false);
            }
            user_temps_[idx] = true;
        }
        write_filament_fields(*slot, info);

        // Stage the override into overrides_ so the edit survives a restart;
        // the lane's own declaration, filed when the edit is committed, is what
        // apply_resolved_lane paints on every subsequent parse.
        helix::ams::stage_user_override(overrides_, slot_index, info, declared);
    }

    spdlog::info("[ACE] Updated slot {} info: {} {}", slot_index, info.material, info.color_name);

    if (override_store_) {
        helix::ams::persist_staged_override(override_store_.get(), mutex_, overrides_, slot_index,
                                            backend_log_tag(), "Override");
    }

    emit_event(kEventSlotChanged, std::to_string(slot_index));
    return AmsErrorHelper::success();
}

AmsError AmsBackendAce::sync_external_identity(int slot_index, const SlotInfo& info) {
    {
        std::lock_guard<std::mutex> lock(mutex_);

        SlotInfo* slot = mutable_slot_locked(slot_index);
        if (!slot) {
            return AmsErrorHelper::invalid_slot(lane_noun(), slot_index, 0);
        }
        // overrides_ is left alone: a synced value lives in memory only, and
        // the next firmware parse overwrites it.
        write_filament_fields(*slot, info);
    }

    spdlog::info("[ACE] Synced slot {} info: {} {}", slot_index, info.material, info.color_name);
    emit_event(kEventSlotChanged, std::to_string(slot_index));
    return AmsErrorHelper::success();
}

void AmsBackendAce::persist_external_identity_impl(int slot_index,
                                                   const helix::ams::Observation& spoolman) {
    const std::string tag = backend_log_tag();
    std::lock_guard<std::mutex> lock(mutex_);
    helix::ams::persist_override_external_identity(override_store_.get(), overrides_, slot_index,
                                                   spoolman, tag);
}

void AmsBackendAce::persist_slot_weight(int slot_index, float remaining_weight_g,
                                        float total_weight_g) {
    const std::string tag = backend_log_tag();
    std::lock_guard<std::mutex> lock(mutex_);
    helix::ams::persist_override_weight(override_store_.get(), overrides_, slot_index,
                                        remaining_weight_g, total_weight_g, tag);
}

AmsError AmsBackendAce::can_set_tool_mapping(int /*tool_number*/, int /*slot_index*/) const {
    return AmsErrorHelper::not_supported("Tool mapping");
}

AmsError AmsBackendAce::set_tool_mapping_impl(int tool_number, int slot_index) {
    return can_set_tool_mapping(tool_number, slot_index);
}

std::vector<int> AmsBackendAce::get_tool_mapping() const {
    return {};
}

// ============================================================================
// Bypass Mode (not supported)
// ============================================================================

void AmsBackendAce::set_discovery(const helix::PrinterDiscovery& discovery) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        bypass_detected_ = helix::detect_bypass_macros(discovery);
        bypass_macro_options_ = helix::bypass_macro_options(discovery);
    }
    set_bypass_macros(helix::resolve_bypass_macros_for(discovery));
    std::lock_guard<std::mutex> lock(mutex_);
    discovered_instances_.clear();
    for (const auto& name : discovery.ace_object_names()) {
        if (name.rfind("ace_instance_", 0) == 0) {
            discovered_instances_.push_back(name);
        }
    }
}

void AmsBackendAce::set_bypass_macros(helix::BypassMacros macros) {
    std::lock_guard<std::mutex> lock(mutex_);
    bypass_on_macro_ = std::move(macros.on);
    bypass_off_macro_ = std::move(macros.off);
    if (ace_pro_enabled_seen_) {
        system_info_.supports_bypass = !bypass_on_macro_.empty() && !bypass_off_macro_.empty();
    }
}

AmsError AmsBackendAce::enable_bypass() {
    std::string gcode;
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (AmsError precondition = check_preconditions(); !precondition) {
            return precondition;
        }
        // Checked before the availability predicate, which folds in a user
        // override that can force the controls on. That override cannot supply
        // a macro, and without one there is nothing to send.
        if (bypass_on_macro_.empty()) {
            return AmsError(AmsResult::WRONG_STATE, "Bypass not supported",
                            lv_tr("This system does not support bypass mode"), "");
        }
        if (!helix::bypass_available_for(system_info_.supports_bypass)) {
            return AmsError(AmsResult::WRONG_STATE, "Bypass not supported",
                            lv_tr("This system does not support bypass mode"), "");
        }
        // The macro refuses a seated tool itself, but execute_gcode is
        // fire-and-forget and reports success before Klipper answers, so a
        // refusal there would reach the user as a toast contradicting a success
        // already shown.
        if (seat_loaded_) {
            return AmsError(AmsResult::WRONG_STATE, "Unload filament first",
                            lv_tr("Filament is still loaded. Unload it before enabling bypass."),
                            "");
        }
        gcode = bypass_on_macro_;
    }

    spdlog::info("[ACE] Enabling bypass mode");
    return execute_gcode(gcode);
}

AmsError AmsBackendAce::disable_bypass() {
    std::string gcode;
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (AmsError precondition = check_preconditions(); !precondition) {
            return precondition;
        }
        if (bypass_off_macro_.empty()) {
            return AmsError(AmsResult::WRONG_STATE, "Bypass not supported",
                            lv_tr("This system does not support bypass mode"), "");
        }
        if (!helix::bypass_available_for(system_info_.supports_bypass)) {
            return AmsError(AmsResult::WRONG_STATE, "Bypass not supported",
                            lv_tr("This system does not support bypass mode"), "");
        }
        gcode = bypass_off_macro_;
    }

    spdlog::info("[ACE] Disabling bypass mode");
    return execute_gcode(gcode);
}

bool AmsBackendAce::is_bypass_active() const {
    std::lock_guard<std::mutex> lock(mutex_);
    // Only the ACE path being switched off means a hand-fed spool. A rig that
    // never publishes the switch is not bypassing, it simply has no switch.
    return ace_pro_enabled_seen_ && !ace_pro_enabled_;
}

// ============================================================================
// Dryer Control
// ============================================================================

DryerInfo AmsBackendAce::get_dryer_info(int unit) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (unit >= 0 && static_cast<size_t>(unit) < dryers_.size()) {
        return dryers_[static_cast<size_t>(unit)];
    }
    return helix::ace::ace_pro_dryer_info();
}

DryerInfo& AmsBackendAce::dryer_locked(int unit) {
    const auto index = static_cast<size_t>(std::max(unit, 0));
    if (dryers_.size() <= index) {
        dryers_.resize(index + 1, helix::ace::ace_pro_dryer_info());
    }
    return dryers_[index];
}

AmsError AmsBackendAce::start_drying(float temp_c, int duration_min, int fan_pct, int unit) {
    auto err = check_preconditions();
    if (!err.success()) {
        return err;
    }

    const auto instance = dryer_instance_argument(unit);
    if (!instance) {
        return AmsErrorHelper::invalid_parameter("no ACE unit " + std::to_string(unit));
    }
    const DryerInfo dryer = get_dryer_info(unit);
    if (auto refusal = helix::ace::check_dryer_request(dryer, temp_c, duration_min)) {
        return *refusal;
    }

    spdlog::info("[ACE] Starting drying on unit {}: {}°C for {} minutes", unit, temp_c,
                 duration_min);

    (void)fan_pct;

    std::string gcode = "ACE_START_DRYING" + *instance;
    gcode += " TEMP=" + std::to_string(static_cast<int>(temp_c)) +
             " DURATION=" + std::to_string(duration_min);
    return execute_gcode(gcode);
}

AmsError AmsBackendAce::stop_drying(int unit) {
    const auto instance = dryer_instance_argument(unit);
    if (!instance) {
        return AmsErrorHelper::invalid_parameter("no ACE unit " + std::to_string(unit));
    }
    spdlog::info("[ACE] Stopping drying on unit {}", unit);
    return execute_gcode("ACE_STOP_DRYING" + *instance);
}

std::optional<std::string> AmsBackendAce::dryer_instance_argument(int unit) const {
    std::lock_guard<std::mutex> lock(mutex_);
    // Without INSTANCE= the fork dries every unit it has; other drivers have one.
    if (!instance_objects_) {
        return std::string();
    }
    if (unit < 0 || static_cast<size_t>(unit) >= system_info_.units.size()) {
        return std::nullopt;
    }
    return " INSTANCE=" + std::to_string(unit);
}

AmsError AmsBackendAce::update_drying(float temp_c, int duration_min, int fan_pct, int unit) {
    auto err = stop_drying(unit);
    if (!err.success()) {
        return err;
    }
    const DryerInfo dryer = get_dryer_info(unit);
    const auto [target_temp, target_duration] =
        helix::ace::restart_request(dryer, temp_c, duration_min);
    return start_drying(target_temp, target_duration, fan_pct, unit);
}

std::vector<DryingPreset> AmsBackendAce::get_drying_presets() const {
    return get_default_drying_presets();
}

// ============================================================================
// Combined ACE Object Parsing (WebSocket subscription path)
// ============================================================================

SlotInfo* AmsBackendAce::mutable_slot_locked(int slot_index) {
    // Caller holds mutex_.
    if (slot_index < 0) {
        return nullptr;
    }
    for (auto& unit : system_info_.units) {
        const int local = slot_index - unit.first_slot_global_index;
        if (local >= 0 && local < static_cast<int>(unit.slots.size())) {
            return &unit.slots[static_cast<size_t>(local)];
        }
    }
    return nullptr;
}

AmsUnit& AmsBackendAce::unit_locked(int unit) {
    // Caller holds mutex_. A unit below this one that has not reported yet holds
    // its four slots, disconnected, so the units after it keep their numbers.
    const auto index = static_cast<size_t>(std::max(unit, 0));
    while (system_info_.units.size() <= index) {
        AmsUnit& added = system_info_.units.emplace_back();
        added.unit_index = static_cast<int>(system_info_.units.size()) - 1;
        added.name = "ACE Pro";
        if (system_info_.units.size() <= index) {
            added.connected = false;
            added.slots.resize(kSlotsPerUnit);
        } else {
            added.connected = true;
        }
    }
    return system_info_.units[index];
}

bool AmsBackendAce::index_units_locked() {
    // Caller holds mutex_.
    bool renumbered = false;
    int next = 0;
    for (auto& unit : system_info_.units) {
        renumbered |= unit.first_slot_global_index != next && unit.slot_count > 0;
        unit.first_slot_global_index = next;
        unit.slot_count = static_cast<int>(unit.slots.size());
        for (size_t i = 0; i < unit.slots.size(); ++i) {
            unit.slots[i].slot_index = static_cast<int>(i);
            unit.slots[i].global_index = next + static_cast<int>(i);
        }
        next += unit.slot_count;
    }
    system_info_.total_slots = next;
    return renumbered;
}

void AmsBackendAce::name_units_locked() {
    // Caller holds mutex_.
    std::map<std::string, int> seen;
    for (size_t i = 0; i < system_info_.units.size(); ++i) {
        const std::string model = i < unit_models_.size() ? unit_models_[i] : std::string();
        auto& display_name = system_info_.units[i].display_name;
        if (model.empty()) {
            display_name.clear();
            continue;
        }
        const int count = ++seen[model];
        display_name = count == 1 ? model : model + " #" + std::to_string(count);
    }
}

void AmsBackendAce::set_unit_action_locked(int unit, AmsAction action) {
    // Caller holds mutex_.
    const auto index = static_cast<size_t>(std::max(unit, 0));
    if (unit_actions_.size() <= index) {
        unit_actions_.resize(index + 1, AmsAction::IDLE);
    }
    unit_actions_[index] = action;
    const auto busy = std::find_if(unit_actions_.begin(), unit_actions_.end(),
                                   [](AmsAction a) { return a != AmsAction::IDLE; });
    system_info_.action = busy == unit_actions_.end() ? AmsAction::IDLE : *busy;
}

void AmsBackendAce::clear_seated_slot_stamp_locked() {
    // Caller holds mutex_.
    if (seated_stamp_slot_ < 0) {
        return;
    }

    SlotInfo* slot = mutable_slot_locked(seated_stamp_slot_);
    // A resized/rebuilt slot vector has already written firmware truth here,
    // so the saved status is stale — only restore over a stamp still visible.
    if (slot != nullptr && slot->status == SlotStatus::LOADED) {
        slot->status = seated_stamp_prev_;
    }

    seated_stamp_slot_ = -1;
    seated_stamp_prev_ = SlotStatus::UNKNOWN;
}

bool AmsBackendAce::apply_seated_slot_stamp_locked() {
    // Caller holds mutex_.
    clear_seated_slot_stamp_locked();

    const bool prev_loaded = system_info_.filament_loaded;
    system_info_.filament_loaded = seat_loaded_ && !path_empty_under_seat_locked();
    const bool changed = system_info_.filament_loaded != prev_loaded;

    if (!system_info_.filament_loaded || system_info_.current_slot < 0) {
        return changed;
    }

    SlotInfo* slot = mutable_slot_locked(system_info_.current_slot);
    if (slot == nullptr) {
        return changed;
    }

    seated_stamp_slot_ = system_info_.current_slot;
    seated_stamp_prev_ = slot->status;
    slot->status = SlotStatus::LOADED;
    return changed;
}

void AmsBackendAce::apply_path_sensors_locked(const json& data) {
    // Caller holds mutex_.
    bool stated = false;
    if (data.contains("rdm_sensor") && data["rdm_sensor"].is_boolean()) {
        rdm_sensor_ = data["rdm_sensor"].get<bool>();
        stated = true;
    }
    if (data.contains("toolhead_sensor") && data["toolhead_sensor"].is_boolean()) {
        toolhead_sensor_ = data["toolhead_sensor"].get<bool>();
        stated = true;
    }
    if (!stated) {
        return;
    }
    path_sensors_seen_ = true;

    // Publish them on the units so the path canvas knows the hardware is
    // there and can draw the sensor nodes. Every unit feeds the one hub.
    for (auto& unit : system_info_.units) {
        unit.has_hub_sensor = true;
        unit.hub_sensor_triggered = rdm_sensor_;
        unit.has_toolhead_sensor = true;
    }
}

int AmsBackendAce::slot_for_tool_locked(int tool) const {
    // Caller holds mutex_.
    if (tool < 0) {
        return -1;
    }
    if (slot_tools_.empty()) {
        return tool < system_info_.total_slots ? tool : -1;
    }
    for (const auto& [slot, slot_tool] : slot_tools_) {
        if (slot_tool == tool) {
            return slot;
        }
    }
    return -1;
}

int AmsBackendAce::tool_for_slot_locked(int slot_index) const {
    // Caller holds mutex_.
    // The fork names every slot's tool; a slot it has not named yet has none,
    // since slot order need not be tool order. Other drivers number them alike.
    const auto it = slot_tools_.find(slot_index);
    if (it != slot_tools_.end()) {
        return it->second;
    }
    return instance_objects_ ? -1 : slot_index;
}

bool AmsBackendAce::apply_target_index_locked(const json& data) {
    // Caller holds mutex_. Absent means unchanged: notify frames carry only
    // changed fields.
    if (data.contains("target_index") && data["target_index"].is_number_integer()) {
        target_index_ = data["target_index"].get<int>();
    }

    const int prev_pending = system_info_.pending_target_slot;
    const AmsAction prev_action = system_info_.action;

    if (target_index_ >= 0 && target_index_ != system_info_.current_tool) {
        system_info_.pending_target_slot = slot_for_tool_locked(target_index_);
        if (path_sensors_seen_ && !rdm_sensor_) {
            swap_hub_cleared_ = true;
        }
        const bool retracting =
            seat_loaded_ && path_sensors_seen_ && rdm_sensor_ && !swap_hub_cleared_;
        // Only an idle hub, or one this target already moved, is driven: an
        // error already says more than the target does. A load sent from here
        // is driven too: its LOADING was set before the driver said anything,
        // and on a swap the seated tool still has to retract first; left
        // standing, it would draw the target loaded through the sensors the
        // outgoing strand still makes.
        const AmsAction action = system_info_.action;
        const bool moving = action == AmsAction::LOADING || action == AmsAction::UNLOADING;
        if (action == AmsAction::IDLE || ((driver_action_ || own_load_) && moving)) {
            system_info_.action = retracting ? AmsAction::UNLOADING : AmsAction::LOADING;
            driver_action_ = true;
        }
    } else {
        system_info_.pending_target_slot = -1;
        if (driver_action_ && (system_info_.action == AmsAction::LOADING ||
                               system_info_.action == AmsAction::UNLOADING)) {
            // A load sent from here stays LOADING until its own ack.
            system_info_.action = own_load_ ? AmsAction::LOADING : AmsAction::IDLE;
        }
        driver_action_ = false;
        swap_hub_cleared_ = false;
    }

    return system_info_.pending_target_slot != prev_pending || system_info_.action != prev_action;
}

bool AmsBackendAce::seat_from_tool_locked(int tool) {
    // Caller holds mutex_.
    if (tool < 0) {
        return seat_from_local_index_locked(-1);
    }
    // A tool no known slot feeds (its unit has not reported yet) is stated
    // with no slot marked. Idempotent — the REST poll restates the index
    // every cycle.
    const int slot = slot_for_tool_locked(tool);
    const bool changed =
        system_info_.current_tool != tool || !seat_loaded_ || system_info_.current_slot != slot;
    seat_loaded_ = true;
    system_info_.current_tool = tool;
    system_info_.current_slot = slot;
    return changed;
}

bool AmsBackendAce::seat_from_local_index_locked(int slot_index) {
    // Caller holds mutex_.
    const int prev_slot = system_info_.current_slot;
    const int prev_tool = system_info_.current_tool;
    const bool prev_loaded = seat_loaded_;

    seat_loaded_ = (slot_index >= 0);
    system_info_.current_slot = slot_index;
    system_info_.current_tool = slot_index;

    return system_info_.current_slot != prev_slot || system_info_.current_tool != prev_tool ||
           seat_loaded_ != prev_loaded;
}

void AmsBackendAce::parse_ace_object(const json& data, int unit_index) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Drop the previous frame's derived seat before the slot loop rewrites
    // statuses, so check_hardware_event_clear and prev_slot_status_ compare
    // firmware against firmware. Re-derived at the bottom of this function.
    clear_seated_slot_stamp_locked();

    // Parse system info (model, firmware)
    if (data.contains("model") && data["model"].is_string()) {
        system_info_.type_name = "ACE";
        // Use model string for unit name if we have the unit
        auto model = data["model"].get<std::string>();
        if (static_cast<size_t>(unit_index) < system_info_.units.size()) {
            system_info_.units[static_cast<size_t>(unit_index)].name = model;
        }
    }

    if (instance_objects_ && data.contains("protocol") && data["protocol"].is_string()) {
        if (unit_models_.size() <= static_cast<size_t>(unit_index)) {
            unit_models_.resize(static_cast<size_t>(unit_index) + 1);
        }
        unit_models_[static_cast<size_t>(unit_index)] =
            ace_model_for_protocol(data["protocol"].get<std::string>());
        name_units_locked();
    }

    if (data.contains("firmware") && data["firmware"].is_string() && unit_index == 0) {
        system_info_.version = data["firmware"].get<std::string>();
    }

    // Parse status string -> AmsAction. The shared vocabulary table is the
    // one AFC and Happy Hare read; ACE's own words (loading/unloading/error,
    // "ready", "drying") resolve to the same actions through it. Each unit
    // states its own, so one unit's "ready" does not end another's load.
    if (data.contains("status") && data["status"].is_string()) {
        set_unit_action_locked(unit_index,
                               ams_action_from_string(data["status"].get<std::string>()));
    }

    // Parse slots array
    if (data.contains("slots") && data["slots"].is_array()) {
        const auto& slots_arr = data["slots"];

        // Sanity check
        if (slots_arr.size() > 16) {
            spdlog::warn("[ACE] Ignoring excessive slot count from get_status: {}",
                         slots_arr.size());
        } else {
            const bool new_unit = static_cast<size_t>(unit_index) >= system_info_.units.size();
            auto& unit = unit_locked(unit_index);
            if (new_unit && data.contains("model") && data["model"].is_string()) {
                unit.name = data["model"].get<std::string>();
            }
            if (new_unit) {
                name_units_locked();
            }
            if (instance_objects_) {
                // Every fork unit feeds the printer's one extruder through the
                // one hub its rdm_sensor watches.
                unit.hub_id = "ace";
            }

            // Resize slots
            if (unit.slots.size() != slots_arr.size()) {
                unit.slots.resize(slots_arr.size());
            }
            const auto link = unit_links_.find(unit_index);
            unit.connected = link == unit_links_.end() || link->second;
            if (index_units_locked()) {
                // Slots moved: the insert edge compares a bay with itself only,
                // so it re-baselines as after a restart.
                prev_slot_status_.clear();
                pending_insert_reads_.clear();
                last_spool_evidence_.clear();
            }

            for (size_t i = 0; i < slots_arr.size(); ++i) {
                const auto& slot_json = slots_arr[i];
                if (!slot_json.is_object())
                    continue;

                auto& slot = unit.slots[i];
                const int idx = slot.global_index;
                note_park_state_locked(idx, slot_json);
                if (slot_json.contains("tool") && slot_json["tool"].is_number_integer()) {
                    slot_tools_[idx] = slot_json["tool"].get<int>();
                    // Published as the slot's tool so a Load with a tool seated
                    // is planned as the toolchange the driver performs, and the
                    // step bar shows its cut and retract.
                    slot.mapped_tool = slot_tools_[idx];
                }
                if (instance_objects_) {
                    slot.extruder_name = "extruder";
                }

                // Parse status via the shared vocabulary map so the object path
                // and the REST fallback path can't drift.
                //
                // Each parsed value is also kept in a local for the lane
                // observations below, rather than read back off `slot`: the
                // override merge at the bottom of this loop rewrites that
                // member in place and leaves it rewritten for every later
                // frame, so an observation sourced from it would file a user's
                // own edit as the hub's memory.
                const bool frame_states_status =
                    slot_json.contains("status") && slot_json["status"].is_string();
                std::optional<bool> observed_present;
                if (frame_states_status) {
                    const SlotStatus status =
                        helix::ace::slot_status_from_string(slot_json["status"].get<std::string>());
                    observed_present = slot_status_reports_filament(status);
                    slot.status = status;
                    note_driver_ready_locked(slot.global_index,
                                             slot_json["status"].get<std::string>());
                }

                // Parse color: ValgACE returns [r, g, b] array
                std::optional<uint32_t> observed_color;
                if (slot_json.contains("color")) {
                    observed_color = helix::ace::parse_slot_color(slot_json["color"]);
                    // A value that carries no colour leaves the bay showing the
                    // last one the hub did state, rather than flipping it to
                    // black - which is a colour ACE really does report.
                    if (observed_color) {
                        slot.color_rgb = *observed_color;
                    }
                }

                // Parse material (e.g., "PLA", "PETG") - see read_slot_material.
                std::optional<std::string> observed_material =
                    helix::ace::read_slot_material(slot_json);
                if (observed_material) {
                    slot.material = *observed_material;
                }

                // Parse SKU if present
                if (slot_json.contains("sku") && slot_json["sku"].is_string()) {
                    // SKU is available but not mapped to SlotInfo currently
                }

                // The rfid field is the bay's tag reader: only an identified
                // read states what is in the bay; without one the hub states its
                // own memory of it (prestonbrown/helixscreen#1710). A read in
                // progress leaves the evidence incomplete, so the insert edge
                // holds its verdict for a later frame.
                const helix::ace::TagRead tag_read = helix::ace::read_tag_state(slot_json);
                helix::ams::SpoolEvidence evidence;
                if (tag_read == helix::ace::TagRead::Identified) {
                    evidence.material = observed_material.value_or(std::string{});
                    evidence.color_rgb = observed_color;
                    evidence.tag_read_complete = true;
                } else if (tag_read == helix::ace::TagRead::NoTag) {
                    evidence.tag_read_complete = true;
                }

                // Insert-edge verdict. Must run BEFORE apply_resolved_lane so
                // the check sees firmware-truth (not the resolved view); once a
                // DifferentSpool clear has run, the lane declares nothing for
                // those fields and apply_resolved_lane leaves them as firmware
                // set them.
                //
                // First observation (no prev_slot_status_ entry) is a
                // BASELINE and must never fire a clear, matching IFS/Snapmaker
                // baseline semantics. Only call the helper when a prior status
                // was already recorded for this slot.
                auto prev_it = prev_slot_status_.find(idx);
                if (prev_it != prev_slot_status_.end()) {
                    check_hardware_event_clear(slot, idx, prev_it->second, slot.status, evidence);
                }
                prev_slot_status_[idx] = slot.status;
                // Remember the reading for the spool now in the bay, and keep
                // it across the empty interval: the reading of the spool that
                // left is the comparison side of the insert rule when the next
                // one arrives. Only a frame carrying a read writes it: a
                // no-read frame states hub memory, not what the reader got
                // off the occupant, and overwriting with it would lose the
                // comparison side while an insert is still pending. Must
                // follow the check, which reads the previous entry.
                if ((slot.status == SlotStatus::AVAILABLE || slot.status == SlotStatus::LOADED) &&
                    evidence.tag_read_complete) {
                    last_spool_evidence_[idx] = evidence;
                }

                // The hub reports an occupancy status and a colour. Neither is
                // an identity reading, so the colour is the hub's own memory of
                // what was last in the bay rather than proof of what is there.
                //
                // A frame that states no status for this bay files no sensed
                // record: one ingest replaces a source's record whole, so an
                // empty one would erase a live reading instead of repeating it.
                // A status ACE does not recognise DOES file one, with `present`
                // unset - the hub saying it does not know is news, and a record
                // whose field is unset is the only way to retract a reading.
                if (frame_states_status) {
                    helix::ams::Observation sensed(helix::ams::ObservationSource::Sensed);
                    sensed.present = observed_present;
                    helix::ams::ingest(lane_id(idx), sensed);
                }

                // The cache record is filed on every pass, with only what this
                // frame stated in it. `slots` is one Klipper status field, so
                // Moonraker sends the array whole or not at all and a frame
                // that describes a bay describes all of it - there is no
                // partial identity for a whole-record write to narrow. It also
                // makes an absent sensed record provably a guard rather than a
                // translation that did not run.
                helix::ams::Observation cache(helix::ams::ObservationSource::VendorCache);
                cache.color_rgb = observed_color;
                cache.material = observed_material;
                helix::ams::ingest(lane_id(idx), cache);
                note_tag_frame_locked(idx, tag_read, slot_json, observed_material, observed_color,
                                      observed_present);

                // Layer user-configured overrides on top of firmware-reported
                // data. Override wins for any non-default field - for ACE
                // that includes color and material, since ACE hardware
                // doesn't carry brand/spool_name/weights at all and the user
                // edit is the authoritative source for color/material too.
                // The REST poll brings every slot through here each
                // kPollIntervalMs, so ACE names no slot to cached_slot_locked().
                apply_resolved_lane(slot, idx);
            }
            if (!slot_tools_.empty()) {
                // The driver names each slot's tool: publish the map in both
                // directions, so the T badges show only where tool and spool
                // numbers part.
                helix::printer::sync_tool_map_from_slots(system_info_);
            }
        }
    }

    // The fork states each unit's link; a frame that does not restate it
    // leaves it as it was.
    if (instance_objects_ && data.contains("connection_state") &&
        data["connection_state"].is_string()) {
        if (const auto linked =
                ace_link_from_state(data["connection_state"].get_ref<const std::string&>())) {
            unit_links_[unit_index] = *linked;
            if (static_cast<size_t>(unit_index) < system_info_.units.size()) {
                system_info_.units[static_cast<size_t>(unit_index)].connected = *linked;
            }
        }
    }

    // Parse dryer state — either outer-key spelling (`dryer` or
    // `dryer_status`), either nested key set, plus the top-level ambient
    // `temp`. See helix::ace::apply_dryer_state.
    DryerInfo& dryer = dryer_locked(unit_index);
    helix::ace::apply_dryer_state(data, dryer);

    AmsUnit* env_unit = static_cast<size_t>(unit_index) < system_info_.units.size()
                            ? &system_info_.units[static_cast<size_t>(unit_index)]
                            : nullptr;

    // Populate per-unit environment data for the environment overlay: the
    // ambient temperature, and humidity where the unit has a sensor (an ACE 2
    // reports it, an ACE Pro reports null). Updated in place: a notify frame
    // carries only the fields that changed, so one without `humidity` keeps it.
    if (env_unit != nullptr && dryer.current_temp_c > 0) {
        if (!env_unit->environment.has_value()) {
            env_unit->environment = EnvironmentData{};
        }
        env_unit->environment->temperature_c = dryer.current_temp_c;
    }
    if (env_unit != nullptr && data.contains("humidity") && data["humidity"].is_number()) {
        if (!env_unit->environment.has_value()) {
            env_unit->environment = EnvironmentData{};
        }
        env_unit->environment->humidity_pct = data["humidity"].get<float>();
        env_unit->environment->has_humidity = true;
    }

    // Derive loaded slot state from slot statuses
    // ValgACE doesn't have a top-level "loaded_slot" — infer from slot status
    // If any slot is "loaded", that's the active one
    if (env_unit != nullptr) {
        for (int i = 0; i < static_cast<int>(env_unit->slots.size()); ++i) {
            // Check the raw JSON for "loaded" status specifically
            if (data.contains("slots") && data["slots"].is_array() &&
                i < static_cast<int>(data["slots"].size())) {
                const auto& sj = data["slots"][static_cast<size_t>(i)];
                if (sj.contains("status") && sj["status"].is_string() &&
                    sj["status"].get<std::string>() == "loaded") {
                    seat_from_local_index_locked(env_unit->first_slot_global_index + i);
                    break;
                }
            }
        }
    }

    // Also handle explicit loaded_slot if present (future compatibility)
    if (data.contains("loaded_slot") && data["loaded_slot"].is_number_integer()) {
        seat_from_local_index_locked(data["loaded_slot"].get<int>());
    }

    // Native Anycubic GoKlipper reports the loaded slot as a
    // "current_filament" string of the form "<unitId>-<localIndex>" (e.g.
    // "0-2" = local slot 2). An empty string or absent field means nothing is
    // loaded — in that case leave current_slot as managed by load/unload logic
    // (do NOT force -1 here).
    if (data.contains("current_filament") && data["current_filament"].is_string()) {
        const std::string cf = data["current_filament"].get<std::string>();
        auto dash = cf.find('-');
        if (!cf.empty() && dash != std::string::npos && dash + 1 < cf.size()) {
            const auto local_index = helix::text_io::parse_leading<int>(cf.substr(dash + 1));
            if (!local_index) {
                spdlog::debug("[ACE] Failed to parse current_filament '{}'", cf);
            } else if (*local_index >= 0) {
                seat_from_local_index_locked(*local_index);
            }
        }
    }

    // Kobra S1 fork: the manager object states the seat ONLY as
    // `current_index`, the global tool index, and states it even when
    // nothing is loaded (-1) — so it must clear the seat too, or a TR/unload
    // would leave the previous slot reading loaded forever. Fourth and last
    // explicit signal; last one wins (#1069).
    if (data.contains("current_index") && data["current_index"].is_number_integer()) {
        manager_states_seat_ = true;
        seat_from_tool_locked(data["current_index"].get<int>());
    }

    apply_path_sensors_locked(data);

    // After the seat, the sensors and the status-derived action, so a
    // toolchange the driver started itself still reads as LOADING over a unit
    // status that stays "ready" through it, in the phase the hub sensor shows.
    apply_target_index_locked(data);

    // The master switch. Presence is the capability, so a rig without one is
    // left reporting no bypass rather than one that is permanently off.
    if (data.contains("ace_pro_enabled") && data["ace_pro_enabled"].is_boolean()) {
        ace_pro_enabled_seen_ = true;
        ace_pro_enabled_ = data["ace_pro_enabled"].get<bool>();
        system_info_.supports_bypass = !bypass_on_macro_.empty() && !bypass_off_macro_.empty();
    }

    if (data.contains("endless_spool_enabled") && data["endless_spool_enabled"].is_boolean()) {
        endless_spool_seen_ = true;
        endless_spool_on_ = data["endless_spool_enabled"].get<bool>();
        system_info_.endless_spool_enabled = endless_spool_on_;
    }
    if (data.contains("endless_spool_match_mode") && data["endless_spool_match_mode"].is_string()) {
        endless_spool_mode_ = data["endless_spool_match_mode"].get<std::string>();
    }

    // All four seated signals (the ValgACE "loaded" scan, loaded_slot, native
    // current_filament, and the fork manager's current_index) have now had
    // their say and arbitrated to one slot; publish that as the slot's own
    // status.
    apply_seated_slot_stamp_locked();
}

const json* AmsBackendAce::select_ace_object(const json& status, std::string* matched_key,
                                             bool require_slots, const json* skip) {
    // One predicate for both selection passes (see the header doc): the
    // subscription-commit pass requires real slot data (a non-empty "slots"
    // array — the exact key parse_ace_object reads — so a manager-only object,
    // the Kobra S1 fork's `ace` with ace_instances/current_index and no slots,
    // falls through to the REST bridge, #1069), while a notify delta needs
    // only a non-empty object the caller has not claimed for itself.
    auto matches = [&](const json& obj) {
        if (!obj.is_object() || obj.empty() || &obj == skip) {
            return false;
        }
        return !require_slots ||
               (obj.contains("slots") && obj["slots"].is_array() && !obj["slots"].empty());
    };

    // Preference order: filament_hub (native GoKlipper), then ace (community
    // ValgACE/BunnyACE), then ace_instance_N (Kobra S1 mainline-Python fork —
    // #1107), so the choice is deterministic.
    if (status.contains("filament_hub") && matches(status["filament_hub"])) {
        if (matched_key)
            *matched_key = "filament_hub";
        return &status["filament_hub"];
    }
    if (status.contains("ace") && matches(status["ace"])) {
        if (matched_key)
            *matched_key = "ace";
        return &status["ace"];
    }
    if (const std::string* instance_key = lowest_ace_instance_key(status, matches)) {
        if (matched_key)
            *matched_key = *instance_key;
        return &status[*instance_key];
    }
    return nullptr;
}

const std::string*
AmsBackendAce::lowest_ace_instance_key(const json& status,
                                       const std::function<bool(const json&)>& predicate) {
    if (!status.is_object()) {
        return nullptr;
    }
    const std::string* best_key = nullptr;
    for (auto it = status.begin(); it != status.end(); ++it) {
        if (it.key().rfind("ace_instance", 0) == 0 && predicate(it.value()) &&
            (best_key == nullptr || it.key() < *best_key)) {
            best_key = &it.key();
        }
    }
    return best_key;
}

std::vector<std::pair<int, const json*>> AmsBackendAce::ace_instance_objects(const json& status) {
    std::vector<std::pair<int, const json*>> found;
    if (!status.is_object()) {
        return found;
    }
    static constexpr std::string_view kPrefix = "ace_instance_";
    for (auto it = status.begin(); it != status.end(); ++it) {
        if (it.key().rfind(kPrefix, 0) != 0 || !it.value().is_object() || it.value().empty()) {
            continue;
        }
        const auto number = helix::text_io::parse_leading<int>(it.key().substr(kPrefix.size()));
        if (!number || *number < 0 || *number >= kMaxUnits) {
            spdlog::debug("[ACE] Ignoring '{}': not an ACE instance this backend shows", it.key());
            continue;
        }
        found.emplace_back(*number, &it.value());
    }
    std::sort(found.begin(), found.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    return found;
}

const json* AmsBackendAce::manager_ace_object(const json& status) {
    // Any slotless `ace` is the manager's half of the frame. A notify delta
    // carries only the fields that changed, so one stating just the endless
    // spool mode, a sensor or target_index names no current_index, and it
    // arrives beside whatever instance delta Klipper batched with it
    // (prestonbrown/helixscreen#1679).
    if (!status.is_object() || !status.contains("ace") || !status["ace"].is_object()) {
        return nullptr;
    }
    const json& ace = status["ace"];
    if (ace.empty()) {
        return nullptr;
    }
    if (ace.contains("slots") && ace["slots"].is_array() && !ace["slots"].empty()) {
        return nullptr; // slot-bearing ValgACE `ace` — the primary, not a manager
    }
    return &ace;
}

// ============================================================================
// REST Fallback (for BunnyACE/DuckACE)
// ============================================================================

void AmsBackendAce::start_rest_fallback() {
    use_rest_fallback_ = true;
    rest_stop_requested_.store(false);
    // Reap any prior thread before move-assignment. Reassigning to a joinable
    // std::thread calls std::terminate WITHOUT an active exception, regardless
    // of whether the loop has already exited (UBZQ94EE-class bug).
    if (rest_polling_thread_.joinable()) {
        rest_polling_thread_.join();
    }
    // EAGAIN under thread exhaustion throws std::system_error ([L083]).
    if (!helix::contain_exceptions("[ACE] Spawning the REST polling thread", [&] {
            rest_polling_thread_ = helix::make_thread(&AmsBackendAce::rest_polling_loop, this);
        })) {
        use_rest_fallback_ = false;
        return;
    }
    spdlog::info("[ACE] REST fallback polling started");
}

void AmsBackendAce::stop_rest_fallback() {
    if (!use_rest_fallback_)
        return;

    rest_stop_requested_.store(true);
    {
        std::lock_guard<std::mutex> lock(rest_stop_mutex_);
        rest_stop_cv_.notify_all();
    }
    if (rest_polling_thread_.joinable()) {
        rest_polling_thread_.join();
    }
    use_rest_fallback_ = false;
    spdlog::debug("[ACE] REST fallback polling stopped");
}

void AmsBackendAce::rest_polling_loop() {
    spdlog::debug("[ACE] REST polling thread started");

    // Fetch system info first
    poll_info();

    while (!rest_stop_requested_.load()) {
        // /server/ace/info is OPTIONAL — best-effort retry until the cap for the
        // model/version it can supply, but its absence is NOT fatal: the Kobra
        // S1 fork ships /status + /slots (which carry model + slots) but NOT
        // /info (#1069). Give up quietly after the cap.
        if (!info_fetched_.load() && info_fetch_failures_.load() < kMaxInfoFetchFailures) {
            poll_info();
        } else if (!info_fetched_.load() && info_fetch_failures_.load() == kMaxInfoFetchFailures) {
            spdlog::info("[ACE] /server/ace/info unavailable after {} attempts — "
                         "using /status + /slots (this fork has no /info endpoint).",
                         kMaxInfoFetchFailures);
            ++info_fetch_failures_;
        }

        // Data endpoints are the source of truth for slots (and model, via the
        // /status envelope) — poll them unconditionally, NOT gated on
        // info_fetched_. Gating here left the fork's backend permanently idle.
        poll_status();
        poll_slots();

        // Interruptible sleep
        std::unique_lock<std::mutex> lock(rest_stop_mutex_);
        rest_stop_cv_.wait_for(lock, std::chrono::milliseconds(kPollIntervalMs),
                               [this] { return rest_stop_requested_.load(); });
    }

    spdlog::debug("[ACE] REST polling thread exiting");
}

void AmsBackendAce::poll_info() {
    if (!api_) {
        return;
    }

    spdlog::debug("[ACE] Polling /server/ace/info");

    struct SyncState {
        std::mutex mtx;
        std::condition_variable cv;
        bool done{false};
    };
    auto state = std::make_shared<SyncState>();

    auto token = op_lifetime_.token();

    api_->rest().call_rest_get("/server/ace/info", [this, state, token](const RestResponse& resp) {
        // L081 Mechanism C: defer member access to main thread. The synchronous
        // waiter (state->cv) MUST be signaled regardless of owner liveness, so
        // we enqueue the parse defer first, then signal state->done + cv on the
        // bg thread before returning. The defer becomes a no-op if the owner
        // has been destroyed; the poll_info caller is unblocked either way.
        token.defer("AmsBackendAce::poll_info_apply", [this, resp]() {
            if (resp.success && resp.data.contains("result")) {
                parse_info_response(resp.data["result"]);
                info_fetched_.store(true);
                info_fetch_failures_ = 0;
            } else {
                // /info is optional (#1069) — a failure here is NOT surfaced to
                // the user. The "bridge not found" error is gated on the DATA
                // endpoints (/status + /slots) failing instead; see poll_status.
                int failures = ++info_fetch_failures_;
                spdlog::debug("[ACE] /server/ace/info attempt {} failed (optional): {}", failures,
                              resp.error);
            }
        });

        {
            std::lock_guard<std::mutex> lock(state->mtx);
            state->done = true;
        }
        state->cv.notify_one();
    });

    // Wait for response (with timeout)
    std::unique_lock<std::mutex> lock(state->mtx);
    state->cv.wait_for(lock, std::chrono::seconds(5), [state] { return state->done; });
}

void AmsBackendAce::poll_status() {
    if (!api_) {
        return;
    }

    spdlog::trace("[ACE] Polling /server/ace/status");

    auto token = op_lifetime_.token();

    api_->rest().call_rest_get("/server/ace/status", [this, token](const RestResponse& resp) {
        // L081 Mechanism C: defer member access (parse_status_response,
        // emit_event) to main thread.
        token.defer("AmsBackendAce::poll_status_apply", [this, resp]() {
            if (resp.success && resp.data.contains("result")) {
                rest_data_ok_.store(true);
                data_fetch_failures_.store(0);
                if (parse_status_response(resp.data["result"])) {
                    emit_event(EVENT_STATE_CHANGED);
                }
            } else {
                int failures = ++data_fetch_failures_;
                spdlog::debug("[ACE] Status poll attempt {} failed: {}", failures, resp.error);
                // Genuinely-missing-bridge case: the DATA endpoints are down and
                // nothing has ever succeeded. Surface it once. A working /slots
                // resets data_fetch_failures_, so this only fires when BOTH
                // /status and /slots are unavailable (#1069).
                if (failures == kMaxDataFetchFailures && !rest_data_ok_.load()) {
                    spdlog::warn("[ACE] Moonraker ACE bridge not responding at /server/ace/status "
                                 "after {} attempts — no ACE data endpoints available.",
                                 failures);
                    emit_event(kEventError,
                               "ACE detected but Moonraker bridge not found. "
                               "Install the ace_status.py component for full ACE support.");
                    helix::ui::queue_update("AmsBackendAce::poll_status", []() {
                        ToastManager::instance().show(
                            ToastSeverity::WARNING,
                            lv_tr("ACE Moonraker bridge not found. Install ace_status.py "
                                  "for full support."),
                            6000);
                    });
                }
            }
        });
    });
}

void AmsBackendAce::poll_slots() {
    if (!api_) {
        return;
    }

    spdlog::trace("[ACE] Polling /server/ace/slots");

    auto token = op_lifetime_.token();

    api_->rest().call_rest_get("/server/ace/slots", [this, token](const RestResponse& resp) {
        // L081 Mechanism C: defer member access (parse_slots_response,
        // emit_event) to main thread.
        token.defer("AmsBackendAce::poll_slots_apply", [this, resp]() {
            if (resp.success && resp.data.contains("result")) {
                // A working /slots is sufficient data — latch data-ok and reset
                // the failure counter so the "bridge not found" error only fires
                // when BOTH data endpoints are down (#1069).
                rest_data_ok_.store(true);
                data_fetch_failures_.store(0);
                if (parse_slots_response(resp.data["result"])) {
                    // REST fallback parses all slots at once — use STATE_CHANGED
                    // (full-sync semantics) rather than SLOT_CHANGED without a
                    // slot_index.
                    emit_event(EVENT_STATE_CHANGED);
                }
            } else {
                spdlog::debug("[ACE] Slots poll failed: {}", resp.error);
            }
        });
    });
}

// ============================================================================
// REST Response Parsing (fallback path)
// ============================================================================

void AmsBackendAce::parse_info_response(const json& data) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (data.contains("model") && data["model"].is_string()) {
        system_info_.type_name = "ACE";
    }

    if (data.contains("version") && data["version"].is_string()) {
        system_info_.version = data["version"].get<std::string>();
    }

    if (data.contains("slot_count") && data["slot_count"].is_number_integer()) {
        int slot_count = data["slot_count"].get<int>();

        if (slot_count < 0 || slot_count > 16) {
            spdlog::warn("[ACE] Ignoring invalid slot_count: {}", slot_count);
            return;
        }

        system_info_.total_slots = slot_count;

        if (system_info_.units.empty()) {
            system_info_.units.emplace_back();
            system_info_.units[0].name = "ACE Pro";
            system_info_.units[0].unit_index = 0;
            system_info_.units[0].connected = true;
        }

        auto& unit = system_info_.units[0];
        unit.slot_count = slot_count;

        if (unit.slots.size() != static_cast<size_t>(slot_count)) {
            unit.slots.resize(static_cast<size_t>(slot_count));
            for (int i = 0; i < slot_count; ++i) {
                unit.slots[static_cast<size_t>(i)].slot_index = i;
                unit.slots[static_cast<size_t>(i)].global_index = i;
                unit.slots[static_cast<size_t>(i)].status = SlotStatus::UNKNOWN;
            }
        }
    }

    spdlog::info("[ACE] Detected: {} v{} with {} slots", system_info_.type_name,
                 system_info_.version, system_info_.total_slots);
}

bool AmsBackendAce::parse_status_response(const json& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    bool changed = false;

    // The fork's /server/ace/status envelope carries model + firmware, so the
    // backend can identify the hardware without /server/ace/info (#1069). Match
    // the object-path key names (`model`, `firmware`).
    if (data.contains("model") && data["model"].is_string()) {
        system_info_.type_name = "ACE";
        auto model = data["model"].get<std::string>();
        if (!system_info_.units.empty()) {
            system_info_.units[0].name = model;
        }
    }
    if (data.contains("firmware") && data["firmware"].is_string()) {
        system_info_.version = data["firmware"].get<std::string>();
    }

    if (data.contains("loaded_slot") && data["loaded_slot"].is_number_integer()) {
        changed |= seat_from_local_index_locked(data["loaded_slot"].get<int>());
    }

    // The fork's /status states the seat through the manager's
    // current_index instead of loaded_slot — same semantics, global tool
    // index with -1 meaning nothing loaded (#1069).
    if (data.contains("ace_manager") && data["ace_manager"].is_object() &&
        data["ace_manager"].contains("current_index") &&
        data["ace_manager"]["current_index"].is_number_integer()) {
        manager_states_seat_ = true;
        apply_path_sensors_locked(data["ace_manager"]);
        changed |= seat_from_tool_locked(data["ace_manager"]["current_index"].get<int>());
    }

    // The ValgACE bridge names the action field `action`; the fork names it
    // `status` ("ready"/etc.) with the same loading/unloading/error
    // vocabulary the object path maps (#1069).
    std::string action_str;
    if (data.contains("action") && data["action"].is_string()) {
        action_str = data["action"].get<std::string>();
    } else if (data.contains("status") && data["status"].is_string()) {
        action_str = data["status"].get<std::string>();
    }
    // The ValgACE bridge names the action field `action`; the fork names it
    // `status` ("ready"/etc.) with the same loading/unloading/error
    // vocabulary the object path maps (#1069). A bridge `action` is
    // authoritative; the fork's fallback reads the UNIT's status, which can
    // stay "ready" through a toolchange (do_load_filament sets LOADING
    // optimistically for exactly that case), so an IDLE resolved from it
    // must not demote an in-flight local load/unload.
    const bool action_from_unit_status = !data.contains("action");
    if (!action_str.empty()) {
        const AmsAction action = ams_action_from_string(action_str);
        const bool demotes_in_flight_op = action_from_unit_status && action == AmsAction::IDLE &&
                                          (system_info_.action == AmsAction::LOADING ||
                                           system_info_.action == AmsAction::UNLOADING);
        if (!demotes_in_flight_op && action != system_info_.action) {
            system_info_.action = action;
            changed = true;
        }
    }

    // After the action parse, for the same reason as the object path.
    if (data.contains("ace_manager") && data["ace_manager"].is_object()) {
        changed |= apply_target_index_locked(data["ace_manager"]);
    }

    helix::ace::apply_dryer_state(data, dryer_locked(0));

    // /status owns loaded_slot but never touches the slot vector; /slots owns
    // the slot vector but carries no seated field. Both ends re-derive the
    // stamp so whichever polled last leaves the two consistent.
    changed |= apply_seated_slot_stamp_locked();

    return changed;
}

bool AmsBackendAce::parse_slots_response(const json& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    bool changed = false;

    if (!data.contains("slots") || !data["slots"].is_array()) {
        return false;
    }

    const auto& slots_data = data["slots"];

    if (slots_data.size() > 16) {
        spdlog::warn("[ACE] Ignoring excessive slot count: {}", slots_data.size());
        return false;
    }

    if (system_info_.units.empty()) {
        system_info_.units.emplace_back();
        system_info_.units[0].name = "ACE Pro";
        system_info_.units[0].unit_index = 0;
        system_info_.units[0].connected = true;
    }

    auto& unit = system_info_.units[0];

    if (unit.slots.size() != slots_data.size()) {
        unit.slots.resize(slots_data.size());
        unit.slot_count = static_cast<int>(slots_data.size());
        system_info_.total_slots = static_cast<int>(slots_data.size());
        changed = true;
    }

    // Un-stamp before the per-slot compare below, or a seated slot would read
    // as changed against firmware's "ready" on every 500 ms poll and emit a
    // STATE_CHANGED event forever.
    clear_seated_slot_stamp_locked();

    for (size_t i = 0; i < slots_data.size(); ++i) {
        const auto& slot_json = slots_data[i];

        if (!slot_json.is_object()) {
            continue;
        }

        auto& slot = unit.slots[i];

        slot.slot_index = static_cast<int>(i);
        slot.global_index = static_cast<int>(i);

        changed |= note_park_state_locked(static_cast<int>(i), slot_json);

        const bool states_status = slot_json.contains("status") && slot_json["status"].is_string();
        std::optional<bool> observed_present;
        if (states_status) {
            // Mirror the object path exactly via the shared vocabulary map —
            // this fork emits `ready` (and can emit `unknown`), which the old
            // empty/available/loaded-only mapping misclassified (#1069).
            SlotStatus status =
                helix::ace::slot_status_from_string(slot_json["status"].get<std::string>());
            observed_present = slot_status_reports_filament(status);
            note_driver_ready_locked(slot.global_index, slot_json["status"].get<std::string>());

            if (status != slot.status) {
                slot.status = status;
                changed = true;
            }
        }

        // What the slot showed before this poll, so a user declaration painted
        // back over the hub's values below is not a change every poll.
        const uint32_t shown_color = slot.color_rgb;
        const std::string shown_material = slot.material;

        // Parse color: handle both hex string and RGB array formats
        std::optional<uint32_t> observed_color;
        if (slot_json.contains("color")) {
            observed_color = helix::ace::parse_slot_color(slot_json["color"]);
            if (observed_color) {
                slot.color_rgb = *observed_color;
            }
        }

        // Material — see read_slot_material (one reader for both producers).
        const std::optional<std::string> material = helix::ace::read_slot_material(slot_json);
        if (material) {
            slot.material = *material;
        }

        // The bridge polls the same hub the subscription reads, so it files the
        // same account on the same lanes. Its values come from this response's
        // own keys rather than from `slot`, which a previous poll's diff has
        // already written - and which, on a rig that once ran the object path,
        // apply_resolved_lane has laid a user's declaration onto.
        const int idx = static_cast<int>(i);
        if (states_status) {
            helix::ams::Observation sensed(helix::ams::ObservationSource::Sensed);
            sensed.present = observed_present;
            helix::ams::ingest(lane_id(idx), sensed);
        }

        helix::ams::Observation cache(helix::ams::ObservationSource::VendorCache);
        cache.color_rgb = observed_color;
        cache.material = material;
        helix::ams::ingest(lane_id(idx), cache);

        apply_hub_temps_locked(idx, slot, slot_json);
        note_tag_frame_locked(idx, helix::ace::read_tag_state(slot_json), slot_json, material,
                              observed_color, observed_present);

        // A user's declaration outranks the hub's memory on this path too.
        apply_resolved_lane(slot, idx);
        if (slot.color_rgb != shown_color || slot.material != shown_material) {
            changed = true;
        }
    }

    // /slots carries no seated field, so re-apply what /status last resolved —
    // otherwise this poll would silently demote the loaded slot to AVAILABLE
    // and take can_unload_from_toolhead() with it.
    apply_seated_slot_stamp_locked();

    return changed;
}

// ============================================================================
// Device Actions
// ============================================================================

std::vector<helix::printer::DeviceSection> AmsBackendAce::get_device_sections() const {
    using helix::printer::device_section;
    std::vector<helix::printer::DeviceSection> sections = {
        device_section("filament_control", 0, TR_NOOP("Manual feed and retract operations")),
        device_section("maintenance", 1, TR_NOOP("Feed assist and bypass macros")),
    };
    std::lock_guard<std::mutex> lock(mutex_);
    if (endless_spool_seen_) {
        sections.push_back(
            device_section("endless_spool", 2, TR_NOOP("Switch to a backup slot on runout")));
    }
    return sections;
}

std::vector<helix::printer::DeviceAction> AmsBackendAce::get_device_actions() const {
    using DA = helix::printer::DeviceAction;
    std::vector<DA> actions = {
        DA::button("ace_manual_feed", "Manual Feed", "filament_control", "",
                   "Feed filament from current slot")
            .with_machine_motion(),
        DA::button("ace_manual_retract", "Manual Retract", "filament_control", "",
                   "Retract filament to current slot")
            .with_machine_motion(),
        DA::toggle("ace_feed_assist_toggle", "Feed Assist", "maintenance", {}, "",
                   "Enable feed assist for active slot during printing"),
    };
    std::lock_guard<std::mutex> lock(mutex_);
    if (endless_spool_seen_) {
        actions.push_back(DA::dropdown("ace_endless_spool_mode", "Match Mode", "endless_spool",
                                       {"exact", "material", "next"}, endless_spool_mode_, "",
                                       "exact: same material and color. material: same material. "
                                       "next: any ready slot, even a different material"));
    }
    // Which macros throw the ACE master switch. The conventional pair is
    // ACE_BYPASS_ON/OFF; an owner whose macros are named differently picks them
    // here from the ones the printer reports.
    if (ace_pro_enabled_seen_ && !bypass_macro_options_.empty()) {
        auto& settings = helix::SettingsManager::instance();
        actions.push_back(DA::dropdown(
            "ace_bypass_on_macro", "Bypass on macro", "maintenance", bypass_macro_options_,
            helix::macro_choice_or_auto(settings.get_ace_bypass_on_macro()), "",
            "Which macro switches the ACE path off for bypass"));
        actions.push_back(DA::dropdown(
            "ace_bypass_off_macro", "Bypass off macro", "maintenance", bypass_macro_options_,
            helix::macro_choice_or_auto(settings.get_ace_bypass_off_macro()), "",
            "Which macro switches the ACE path back on"));
    }
    return actions;
}

std::optional<std::string> AmsBackendAce::manual_slot_argument() const {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (instance_objects_) {
            if (system_info_.current_tool < 0) {
                return std::nullopt;
            }
            return "T=" + std::to_string(system_info_.current_tool);
        }
    }
    int slot = get_current_slot();
    if (slot < 0)
        slot = 0;
    return "INDEX=" + std::to_string(slot);
}

AmsError AmsBackendAce::do_execute_device_action(const std::string& action_id,
                                                 const std::any& value) {
    const bool manual_slot_action = action_id == "ace_manual_feed" ||
                                    action_id == "ace_manual_retract" ||
                                    action_id == "ace_feed_assist_toggle";
    std::string slot_arg;
    if (manual_slot_action) {
        const auto arg = manual_slot_argument();
        if (!arg) {
            return AmsError(AmsResult::WRONG_STATE, "No tool loaded",
                            lv_tr("Load a tool before feeding or retracting"), "");
        }
        slot_arg = *arg;
    }

    if (action_id == "ace_manual_feed") {
        static constexpr int kManualFeedLength = 50;
        static constexpr int kManualFeedSpeed = 50;
        return execute_gcode("ACE_FEED " + slot_arg +
                             " LENGTH=" + std::to_string(kManualFeedLength) +
                             " SPEED=" + std::to_string(kManualFeedSpeed));
    }

    if (action_id == "ace_manual_retract") {
        static constexpr int kManualRetractLength = 50;
        static constexpr int kManualRetractSpeed = 50;
        return execute_gcode("ACE_RETRACT " + slot_arg +
                             " LENGTH=" + std::to_string(kManualRetractLength) +
                             " SPEED=" + std::to_string(kManualRetractSpeed));
    }

    if (action_id == "ace_feed_assist_toggle") {
        const auto [enable, err] = printer::device_action_value<bool>(value, "feed assist state");
        if (!err) {
            return err;
        }
        if (enable) {
            return execute_gcode("ACE_ENABLE_FEED_ASSIST " + slot_arg);
        } else {
            return execute_gcode("ACE_DISABLE_FEED_ASSIST " + slot_arg);
        }
    }

    if (action_id == "ace_bypass_on_macro" || action_id == "ace_bypass_off_macro") {
        const auto [chosen, err] = printer::device_action_macro_choice(value);
        if (!err) {
            return err;
        }
        auto& settings = helix::SettingsManager::instance();
        if (action_id == "ace_bypass_on_macro") {
            settings.set_ace_bypass_on_macro(chosen);
        } else {
            settings.set_ace_bypass_off_macro(chosen);
        }
        helix::BypassMacros detected;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            detected = bypass_detected_;
        }
        set_bypass_macros(helix::resolve_bypass_macros(detected, settings.get_ace_bypass_on_macro(),
                                                       settings.get_ace_bypass_off_macro()));
        emit_event(EVENT_STATE_CHANGED);
        return AmsErrorHelper::success();
    }

    if (action_id == "ace_endless_spool_mode") {
        const auto [mode, err] = printer::device_action_value<std::string>(value, "match mode");
        if (!err) {
            return err;
        }
        if (mode != "exact" && mode != "material" && mode != "next") {
            return AmsErrorHelper::invalid_parameter("Unknown endless spool match mode");
        }
        return execute_gcode("ACE_SET_ENDLESS_SPOOL_MODE MODE=" + mode);
    }

    return AmsErrorHelper::not_supported("Unknown ACE action: " + action_id);
}

// ============================================================================
// Endless Spool
// ============================================================================

helix::printer::EndlessSpoolCapabilities AmsBackendAce::get_endless_spool_capabilities() const {
    using namespace helix::printer;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!endless_spool_seen_) {
        return {};
    }
    EndlessSpoolEnabled enabled = EndlessSpoolEnabled::Off;
    if (endless_spool_on_) {
        enabled = endless_spool_config_locked().empty() ? EndlessSpoolEnabled::OnWithoutBackup
                                                        : EndlessSpoolEnabled::On;
    }
    return {.availability = EndlessSpoolAvailability::Available,
            .enabled = enabled,
            .editability = EndlessSpoolEditability::ReadOnly,
            .restriction = EndlessSpoolRestriction::FirmwareManaged,
            .switchable = true,
            .provider = {}};
}

AmsError AmsBackendAce::apply_endless_spool_enabled(bool enabled) {
    return send_endless_spool_switch(enabled ? "ACE_ENABLE_ENDLESS_SPOOL"
                                             : "ACE_DISABLE_ENDLESS_SPOOL");
}

helix::printer::EndlessSpoolConfig AmsBackendAce::get_endless_spool_config() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return endless_spool_config_locked();
}

void AmsBackendAce::note_driver_ready_locked(int global_index, const std::string& driver_status) {
    if (driver_status == "ready") {
        driver_ready_slots_.insert(global_index);
    } else {
        driver_ready_slots_.erase(global_index);
    }
}

helix::printer::EndlessSpoolConfig AmsBackendAce::endless_spool_config_locked() const {
    // Mirrors the driver's find_exact_match: only slots the driver calls
    // "ready" are swap targets, materials compare case- and edge-space-
    // insensitively, and an unknown material never matches in exact or
    // material mode, across every unit's slots.
    if (!endless_spool_seen_ || !endless_spool_on_ || system_info_.units.empty()) {
        return {};
    }
    const bool by_material = endless_spool_mode_ == "material";
    const bool by_next = endless_spool_mode_ == "next";
    auto normalized = [](std::string m) {
        auto not_space = [](unsigned char c) { return !std::isspace(c); };
        m.erase(m.begin(), std::find_if(m.begin(), m.end(), not_space));
        m.erase(std::find_if(m.rbegin(), m.rend(), not_space).base(), m.end());
        std::transform(m.begin(), m.end(), m.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        return m;
    };
    std::map<std::tuple<std::string, uint32_t>, int> ids;
    std::vector<int> group_ids;
    for (const auto& unit : system_info_.units) {
        for (const auto& slot : unit.slots) {
            if (slot.global_index < 0 || !driver_ready_slots_.count(slot.global_index)) {
                continue;
            }
            const std::string material = normalized(slot.material);
            if (!by_next && (material.empty() || material == "unknown")) {
                continue;
            }
            if (group_ids.size() <= static_cast<size_t>(slot.global_index)) {
                group_ids.resize(static_cast<size_t>(slot.global_index) + 1, -1);
            }
            auto key = std::make_tuple(by_next ? std::string() : material,
                                       (by_next || by_material) ? 0u : slot.color_rgb);
            auto [it, inserted] = ids.try_emplace(key, static_cast<int>(ids.size()));
            group_ids[static_cast<size_t>(slot.global_index)] = it->second;
        }
    }
    return helix::printer::endless_spool_config_from_groups(group_ids);
}

// ============================================================================
// Slot Override Layering (shared FilamentSlotOverrideStore)
// ============================================================================

void AmsBackendAce::check_hardware_event_clear(SlotInfo& slot, int slot_index, SlotStatus prev,
                                               SlotStatus curr,
                                               const helix::ams::SpoolEvidence& inserted) {
    // "A spool was put in" is the status transition EMPTY -> present
    // (AVAILABLE / LOADED); what that insert does to the record is the insert
    // rule's verdict on the two tag readings. A LOADED -> EMPTY transition is
    // not an insert, and UNKNOWN is "no signal" on either side; callers must
    // only invoke this helper AFTER a valid prior observation has been
    // recorded (caller handles the baseline skip).
    const bool was_empty = (prev == SlotStatus::EMPTY);
    const bool is_present = (curr == SlotStatus::AVAILABLE || curr == SlotStatus::LOADED);
    if (!is_present) {
        // A bay that empties with an insert still waiting on its tag read
        // drops the insert: the spool left before the reader ever stated
        // one, and there is nothing in the bay to ask about.
        pending_insert_reads_.erase(slot_index);
        return;
    }
    if (!was_empty) {
        // Not an edge. The one thing a non-edge frame still owes is a
        // pending insert: the bay reported the spool on an earlier frame and
        // this one either carries the read it was waiting for, or is another
        // pass with none.
        auto pit = pending_insert_reads_.find(slot_index);
        if (pit == pending_insert_reads_.end())
            return;
        if (inserted.tag_read_complete) {
            judge_insert_locked(slot, slot_index, inserted);
            pending_insert_reads_.erase(pit);
        } else if (++pit->second >= kAcePendingReadParsePasses) {
            // The read never landed. An empty evidence is NoEvidence under
            // the rule, which offers the notice rather than clearing.
            judge_insert_locked(slot, slot_index, helix::ams::SpoolEvidence{});
            pending_insert_reads_.erase(pit);
        }
        return;
    }

    // EMPTY -> present, the insert edge. A frame carrying no read arms
    // rather than judges: the read can land frames later, and judging on
    // nothing would file every insert as no-evidence.
    if (!inserted.tag_read_complete) {
        spdlog::debug("[ACE] Slot {} inserted; tag read not landed yet, holding the verdict",
                      slot_index);
        pending_insert_reads_[slot_index] = 0;
        return;
    }
    judge_insert_locked(slot, slot_index, inserted);
}

void AmsBackendAce::judge_insert_locked(SlotInfo& slot, int slot_index,
                                        const helix::ams::SpoolEvidence& inserted) {
    std::optional<helix::ams::SpoolEvidence> before;
    if (const auto prev_reading = last_spool_evidence_.find(slot_index);
        prev_reading != last_spool_evidence_.end()) {
        before = prev_reading->second;
    }
    const auto verdict = helix::ams::classify_insert(before, inserted);

    if (verdict == helix::ams::InsertVerdict::NoEvidence) {
        // An insert the hardware read nothing about: the bay said no rfid, or
        // the previous occupant left no reading. Nothing contradicts the
        // record and nothing confirms it either, so ask rather than clear.
        // offer_clear_after_unverified_insert() re-checks every guard
        // (print-feeding lane, lane with nothing to clear) on the UI thread.
        spdlog::debug("[ACE] Slot {} inserted with no comparable tag reading; "
                      "offering the same-spool notice",
                      slot_index);
        helix::ui::queue_update("AmsBackendAce::judge_insert_locked", [slot_index]() {
            helix::ui::offer_clear_after_unverified_insert(slot_index);
        });
        return;
    }
    if (verdict != helix::ams::InsertVerdict::DifferentSpool) {
        spdlog::debug("[ACE] Slot {} inserted with the same tag reading; record stands",
                      slot_index);
        return;
    }

    auto ovr_it = overrides_.find(slot_index);
    if (ovr_it == overrides_.end()) {
        spdlog::debug("[ACE] Slot {} inserted with a different tag reading; "
                      "no override to clear",
                      slot_index);
        return;
    }

    spdlog::info("[ACE] Slot {} inserted with a different tag reading; clearing override",
                 slot_index);

    // Delegate erase + field reset + clear_async to the shared helper so
    // hardware-event clears and user-initiated clears share one field-reset
    // policy. Caller already holds mutex_.
    (void)ovr_it;
    clear_override_locked(slot_index, &slot);
    forget_user_temps_locked(slot_index);
}

void AmsBackendAce::clear_slot_override(int slot_index) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto* slot = mutable_slot_locked(slot_index);
        if (!slot) {
            spdlog::warn("{} clear_slot_override: no slot entry for index {}", backend_log_tag(),
                         slot_index);
            return;
        }
        spdlog::info("{} Slot {} override cleared by user request", backend_log_tag(), slot_index);
        clear_override_locked(slot_index, slot);
        forget_user_temps_locked(slot_index);
    }

    emit_event(kEventSlotChanged, std::to_string(slot_index));
}

} // namespace helix

#endif // HELIX_HAS_ACE
