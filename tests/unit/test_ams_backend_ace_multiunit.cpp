// Copyright (C) 2025-2026 356C LLC
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Several ACE units on the Kobra S1 fork (ace_instance_0, ace_instance_1, ...):
// each instance is a unit of its own, and a slot's global index is its tool.

#include "ui_update_queue.h"

#include "../test_helpers/ace_backend_test_helper.h"
#include "ams_backend_ace.h"
#include "ams_types.h"
#include "display_numbering.h"
#include "fake_moonraker_client.h"
#include "filament_op_dispatch.h"
#include "filament_op_execute.h"
#include "lvgl_ui_test_fixture.h"
#include "printer_discovery.h"
#include "ui/ams_drawing_utils.h"

#include <functional>
#include <json.hpp> // nlohmann/json from libhv
#include <string>
#include <vector>

#include "../catch_amalgamated.hpp"

using helix::AmsAction;
using helix::PathSegment;
using helix::SlotStatus;
using json = nlohmann::json;
using namespace ace_fixtures;

namespace {

constexpr uint32_t kUnit0Slot0Color = 0x00E676u; // make_kobra_slots_array, slot 0
constexpr uint32_t kUnit1Color = 0x3F51B5u;      // make_kobra_second_unit_object

void send_frame(AmsBackendAceTestHelper& helper, const json& frame) {
    helper.test_handle_status_update({{"params", json::array({frame, 6000.0})}});
}

json manager_delta(const json& fields) {
    json frame = json::object();
    frame["ace"] = fields;
    return frame;
}

/// The bootstrap of a rig with @p unit_count ACE units, unit N's tools at
/// 4N..4N+3 (ACEPRO numbers tools across its instances in instance order).
/// Unit 0 and the first @p ace_pro_units - 1 further units are ACE Pros, the
/// rest ACE 2s. Like Moonraker, the answer holds only the objects the query
/// asked for.
/// @p edit, when given, changes the printer's full status before it answers.
void start_units(AmsBackendAceTestHelper& helper, helix::test::FakeMoonrakerClient& client,
                 int unit_count, int current_index, int ace_pro_units = 1,
                 const std::function<void(json&)>& edit = nullptr) {
    helper.test_on_started();
    REQUIRE_FALSE(client.rpc_calls.empty());
    const auto& call = client.rpc_calls.back();
    REQUIRE(call.method == "printer.objects.query");
    REQUIRE(call.success_cb);
    json response = make_kobra_objects_query_response(current_index);
    json& status = response["result"]["status"];
    status["ace"]["ace_instances"] = unit_count;
    for (int unit = 1; unit < unit_count; ++unit) {
        json instance = make_kobra_second_unit_object();
        instance["instance"] = unit;
        if (unit < ace_pro_units) {
            instance["protocol"] = "ace1_json";
        }
        for (int i = 0; i < 4; ++i) {
            instance["slots"][i]["tool"] = 4 * unit + i;
        }
        status["ace_instance_" + std::to_string(unit)] = instance;
    }
    if (edit) {
        edit(status);
    }
    json answered = json::object();
    for (const auto& [name, value] : status.items()) {
        if (call.params["objects"].contains(name)) {
            answered[name] = value;
        }
    }
    status = answered;
    call.success_cb(response);
    helix::ui::UpdateQueue::instance().drain();
}

/// What objects.list reports for ACEPRO with @p unit_count units.
helix::PrinterDiscovery ace_discovery(int unit_count) {
    json objects = json::array({"ace", "extruder", "heater_bed"});
    for (int unit = 0; unit < unit_count; ++unit) {
        objects.push_back("ace_instance_" + std::to_string(unit));
    }
    helix::PrinterDiscovery discovery;
    discovery.parse_objects(objects);
    return discovery;
}

} // namespace

TEST_CASE_METHOD(LVGLTestFixture, "ACE shows each fork instance as its own unit",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    start_units(helper, client, 2, -1);

    const auto info = helper.get_test_system_info();
    REQUIRE(info.units.size() == 2);
    CHECK(info.total_slots == 8);
    CHECK(info.units[0].first_slot_global_index == 0);
    CHECK(info.units[1].first_slot_global_index == 4);
    CHECK(info.units[1].slot_count == 4);
    CHECK(info.units[1].unit_index == 1);
    for (int i = 0; i < 4; ++i) {
        CHECK(info.units[1].slots[static_cast<size_t>(i)].global_index == 4 + i);
    }

    // Tools 4-7 carry the second unit's spools; tools 0-3 keep the first's.
    CHECK(helper.get_slot_info(5).material == "ABS");
    CHECK(helper.get_slot_info(5).color_rgb == kUnit1Color);
    CHECK(helper.get_slot_info(0).material == "PLA");
    CHECK(helper.get_slot_info(0).color_rgb == kUnit0Slot0Color);

    // Each unit reports its own environment.
    REQUIRE(info.units[0].environment.has_value());
    REQUIRE(info.units[1].environment.has_value());
    CHECK(info.units[0].environment->temperature_c == 34.0f);
    CHECK(info.units[1].environment->temperature_c == 51.0f);
    CHECK(info.units[1].environment->humidity_pct == 61.0f);
}

TEST_CASE_METHOD(LVGLTestFixture, "ACE units all feed the one toolhead through one hub",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    start_units(helper, client, 2, -1);

    const auto info = helper.get_test_system_info();
    const ams_draw::SystemToolLayout layout = ams_draw::compute_system_tool_layout(info, &helper);
    REQUIRE(layout.units.size() == 2);
    CHECK(layout.total_physical_tools == 1);
    CHECK(layout.units[0].tool_count == 1);
    CHECK(layout.units[1].tool_count == 1);
    CHECK(layout.units[1].first_physical_tool == layout.units[0].first_physical_tool);
    CHECK(layout.hub_groups.size() == 1);
    CHECK(layout.units[1].hub_group == layout.units[0].hub_group);
}

TEST_CASE_METHOD(LVGLTestFixture, "ACE with three units numbers the third unit's slots 8-11",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    start_units(helper, client, 3, 10);

    const auto info = helper.get_test_system_info();
    REQUIRE(info.units.size() == 3);
    CHECK(info.total_slots == 12);
    CHECK(info.units[2].first_slot_global_index == 8);
    CHECK(info.current_slot == 10);
    CHECK(helper.get_slot_info(10).status == SlotStatus::LOADED);
    CHECK(helper.get_slot_info(10).material == "ABS");
}

TEST_CASE_METHOD(LVGLTestFixture, "ACE second-unit delta updates the second unit only",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    start_units(helper, client, 2, -1);

    json slots = make_kobra_second_unit_object()["slots"];
    slots[0]["material"] = "PETG";
    slots[0]["color"] = json::array({255, 0, 0});
    json frame = json::object();
    frame["ace_instance_1"] = json{{"slots", slots}, {"temp", 52}};
    send_frame(helper, frame);

    CHECK(helper.get_slot_info(4).material == "PETG");
    CHECK(helper.get_slot_info(4).color_rgb == 0xFF0000u);
    CHECK(helper.get_dryer_info(1).current_temp_c == 52.0f);
    // Sibling: unit 0 is untouched.
    CHECK(helper.get_slot_info(0).material == "PLA");
    CHECK(helper.get_dryer_info(0).current_temp_c == 34.0f);
}

// T0 seated, a toolchange to T5 on the second unit, in the order the fork
// reported it on a two-unit Kobra S1: target_index first, the outgoing strand
// retracting past the toolhead and hub sensors, unit 1 feeding until both
// sensors are made again, then current_index.
TEST_CASE_METHOD(LVGLTestFixture, "ACE toolchange to a tool on the second unit",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    start_units(helper, client, 2, 0);
    REQUIRE(helper.get_test_system_info().current_slot == 0);

    send_frame(helper, manager_delta({{"target_index", 5}, {"toolchange_in_progress", true}}));
    {
        const auto info = helper.get_test_system_info();
        CHECK(info.action == AmsAction::UNLOADING);
        CHECK(info.pending_target_slot == 5);
        // The outgoing lane retracts first.
        CHECK(info.path_active_slot() == 0);
    }

    send_frame(helper, manager_delta({{"toolhead_sensor", false}}));
    send_frame(helper, manager_delta({{"rdm_sensor", false}}));
    {
        const auto info = helper.get_test_system_info();
        CHECK(info.action == AmsAction::LOADING);
        CHECK(info.path_active_slot() == 5);
    }

    json feeding = make_kobra_second_unit_object()["slots"];
    feeding[1]["status"] = "feeding";
    json unit1_frame = json::object();
    unit1_frame["ace_instance_1"] = json{{"status", "busy"}, {"slots", feeding}};
    send_frame(helper, unit1_frame);
    send_frame(helper, manager_delta({{"rdm_sensor", true}}));
    send_frame(helper, manager_delta({{"toolhead_sensor", true}}));
    {
        const auto info = helper.get_test_system_info();
        CHECK(info.action == AmsAction::LOADING);
        CHECK(info.path_active_slot() == 5);
        CHECK(helper.get_slot_info(0).material == "PLA");
    }

    send_frame(helper, manager_delta({{"current_index", 5}, {"target_index", -1}}));
    const auto info = helper.get_test_system_info();
    CHECK(info.action == AmsAction::IDLE);
    CHECK(info.current_slot == 5);
    CHECK(info.current_tool == 5);
    CHECK(info.filament_loaded);
    CHECK(helper.get_slot_info(5).status == SlotStatus::LOADED);
    CHECK(helper.get_slot_info(0).status != SlotStatus::LOADED);
    CHECK(helper.get_filament_segment() == PathSegment::NOZZLE);
    CHECK(helper.get_slot_filament_segment(5) == PathSegment::NOZZLE);
}

// A notify frame carries only the fields that changed: a unit's frame for a
// slot moving says nothing about its humidity, which still stands.
TEST_CASE_METHOD(LVGLTestFixture, "ACE unit keeps its humidity through a frame without it",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    start_units(helper, client, 2, 0);
    REQUIRE(helper.get_test_system_info().units[1].environment->has_humidity);

    json moving = make_kobra_second_unit_object()["slots"];
    moving[1]["status"] = "feeding";
    json frame = json::object();
    frame["ace_instance_1"] = json{{"status", "busy"}, {"slots", moving}};
    send_frame(helper, frame);

    const auto env = helper.get_test_system_info().units[1].environment;
    REQUIRE(env.has_value());
    CHECK(env->has_humidity);
    CHECK(env->humidity_pct == 61.0f);
    CHECK(env->temperature_c == 51.0f);
}

// A toolchange started from this screen: the load sets LOADING before the
// driver says anything, then the driver names its target while the seated
// tool is still in the toolhead. The path stays on the outgoing tool.
TEST_CASE_METHOD(LVGLTestFixture, "ACE load started here retracts the seated tool first",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    start_units(helper, client, 2, 7);
    REQUIRE(helper.get_test_system_info().current_slot == 7);

    helper.set_running(true);
    REQUIRE(helper.load_filament(4).success());
    send_frame(helper, manager_delta({{"target_index", 4}, {"toolchange_in_progress", true}}));

    const auto info = helper.get_test_system_info();
    CHECK(info.pending_target_slot == 4);
    CHECK(info.action == AmsAction::UNLOADING);
    CHECK(info.path_active_slot() == 7);
}

// The driver names each slot's tool, so a Load from the screen with a tool
// seated is planned as the toolchange it is: the step bar then shows the cut
// and retract before the feed.
TEST_CASE_METHOD(LVGLTestFixture, "ACE load with a tool seated is planned as a toolchange",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);

    SECTION("T7 seated: loading slot 3 changes to T3") {
        start_units(helper, client, 2, 7);
        helix::AmsSystemInfo sys;
        const auto caps = helix::ui::read_backend_caps(&helper, sys, 3);
        const auto plan = helix::ui::plan_load(sys, caps, 3, false, false);
        CHECK(plan.ams_call == helix::ui::AmsCall::ChangeTool);
        CHECK(plan.ams_arg == 3);
    }

    SECTION("nothing seated: a plain load") {
        start_units(helper, client, 2, -1);
        helix::AmsSystemInfo sys;
        const auto caps = helix::ui::read_backend_caps(&helper, sys, 3);
        const auto plan = helix::ui::plan_load(sys, caps, 3, false, false);
        CHECK(plan.ams_call == helix::ui::AmsCall::Load);
        CHECK(plan.ams_arg == 3);
    }
}

// A unit whose ACE has not reported its slots yet (its USB reconnecting) keeps
// its place: the units after it keep their slot numbers when it reports.
TEST_CASE_METHOD(LVGLTestFixture, "ACE unit reporting late keeps the other units' slots",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    start_units(helper, client, 2, -1, 1,
                [](json& status) { status["ace_instance_0"].erase("slots"); });
    {
        const auto info = helper.get_test_system_info();
        REQUIRE(info.units.size() == 2);
        CHECK_FALSE(info.units[0].connected);
        CHECK(info.units[1].first_slot_global_index == 4);
        CHECK(helper.get_slot_info(5).color_rgb == kUnit1Color);
    }

    json frame = json::object();
    frame["ace_instance_0"] = make_kobra_instance_object();
    send_frame(helper, frame);

    const auto info = helper.get_test_system_info();
    CHECK(info.units[0].connected);
    CHECK(info.units[1].first_slot_global_index == 4);
    CHECK(helper.get_slot_info(5).color_rgb == kUnit1Color);
    CHECK(helper.get_slot_info(0).color_rgb == kUnit0Slot0Color);
}

// A restart drops the session's commands with it: a load sent before it does
// not keep a later driver toolchange LOADING.
TEST_CASE_METHOD(LVGLTestFixture, "ACE restart during a load drops the load",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    start_units(helper, client, 2, 7);
    helper.set_running(true);
    REQUIRE(helper.load_filament(4).success());

    start_units(helper, client, 2, 7);
    CHECK(helper.get_test_system_info().action == AmsAction::IDLE);

    send_frame(helper, manager_delta({{"target_index", 4}, {"toolchange_in_progress", true}}));
    send_frame(helper, manager_delta({{"current_index", 4}, {"target_index", -1}}));
    CHECK(helper.get_test_system_info().action == AmsAction::IDLE);
}

TEST_CASE_METHOD(LVGLTestFixture, "ACE ignores an instance number past the unit limit",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    start_units(helper, client, 2, -1);

    json frame = json::object();
    frame["ace_instance_100000"] = make_kobra_second_unit_object();
    send_frame(helper, frame);

    CHECK(helper.get_test_system_info().units.size() == 2);
}

// On the fork a slot's tool is the driver's to name: a slot it has not named
// yet is not loaded under a guessed number, which could be another unit's tool.
TEST_CASE_METHOD(LVGLTestFixture, "ACE refuses a load the driver named no tool for",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    start_units(helper, client, 2, -1, 1,
                [](json& status) { status["ace_instance_1"]["slots"][1].erase("tool"); });
    helper.set_running(true);

    CHECK_FALSE(helper.load_filament(5).success());
    CHECK(helper.captured_gcodes.empty());
}

TEST_CASE_METHOD(LVGLTestFixture, "ACE dryer refuses a unit the fork does not have",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    start_units(helper, client, 2, -1);
    helper.set_running(true);

    CHECK_FALSE(helper.start_drying(50.0f, 60, -1, -1).success());
    CHECK_FALSE(helper.start_drying(50.0f, 60, -1, 2).success());
    CHECK_FALSE(helper.stop_drying(2).success());
    CHECK(helper.captured_gcodes.empty());
}

// The driver retracts the seated tool before it feeds another, so a load sent
// from here with a tool seated starts as the unload it first is.
TEST_CASE_METHOD(LVGLTestFixture, "ACE load with a tool seated starts by unloading",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    start_units(helper, client, 2, 7);
    helper.set_running(true);

    REQUIRE(helper.load_filament(4).success());
    const auto info = helper.get_test_system_info();
    CHECK(info.action == AmsAction::UNLOADING);
    CHECK(info.path_active_slot() == 7);
}

// The tool map states what the per-spool T badges would add: with every tool
// on the slot of its own number they repeat the spool number and stay hidden.
TEST_CASE_METHOD(LVGLTestFixture, "ACE publishes the driver's tool map", "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);

    SECTION("tools numbered like the slots: the badges add nothing") {
        start_units(helper, client, 2, -1);
        const auto info = helper.get_test_system_info();
        REQUIRE(info.tool_to_slot_map.size() == 8);
        CHECK(info.tool_to_slot_map[5] == 5);
        CHECK_FALSE(
            helix::ui::tool_badges_add_information(info.tool_to_slot_map, info.total_slots));
    }

    SECTION("a tool_base: the badges name the driver's tools") {
        start_units(helper, client, 1, -1, 1, [](json& status) {
            for (int i = 0; i < 4; ++i) {
                status["ace_instance_0"]["slots"][i]["tool"] = 4 + i;
            }
        });
        const auto info = helper.get_test_system_info();
        REQUIRE(info.tool_to_slot_map.size() > 6);
        CHECK(info.tool_to_slot_map[6] == 2);
        CHECK(helix::ui::tool_badges_add_information(info.tool_to_slot_map, info.total_slots));
    }
}

// The fork states each unit's link; a unit that is not connected shows as
// disconnected, and a frame that does not restate the link leaves it be.
TEST_CASE_METHOD(LVGLTestFixture, "ACE shows the fork's per-unit connection",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    start_units(helper, client, 2, -1, 1, [](json& status) {
        status["ace_instance_1"]["connection_state"] = "initializing";
    });
    auto unit_connected = [&helper](int unit) {
        return helper.get_test_system_info().units[static_cast<size_t>(unit)].connected;
    };
    auto link = [&helper](const char* state) {
        json frame = json::object();
        frame["ace_instance_1"] = json{{"connection_state", state}};
        send_frame(helper, frame);
    };
    CHECK(unit_connected(0));
    CHECK_FALSE(unit_connected(1));

    link("connected");
    CHECK(unit_connected(1));

    for (const char* state : {"initializing", "connecting", "reconnecting", "disabled"}) {
        link("connected");
        link(state);
        INFO("connection_state " << state);
        CHECK_FALSE(unit_connected(1));
    }

    SECTION("a slots frame that does not restate the link keeps it") {
        link("reconnecting");
        json frame = json::object();
        frame["ace_instance_1"] = json{{"slots", make_kobra_second_unit_object()["slots"]}};
        send_frame(helper, frame);
        CHECK_FALSE(unit_connected(1));
    }

    SECTION("a state this backend does not know claims nothing") {
        link("connected");
        link("unknown");
        CHECK(unit_connected(1));
    }
}

// A guard the per-unit status words must keep: one unit still reporting
// "ready" does not end the other unit's "loading".
TEST_CASE_METHOD(LVGLTestFixture, "ACE idle unit does not clear another unit's action",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    start_units(helper, client, 2, -1);

    json frame = json::object();
    frame["ace_instance_0"] = json{{"status", "loading"}};
    frame["ace_instance_1"] = json{{"status", "ready"}};
    send_frame(helper, frame);
    CHECK(helper.get_test_system_info().action == AmsAction::LOADING);

    SECTION("and the unit's own ready ends it") {
        json done = json::object();
        done["ace_instance_0"] = json{{"status", "ready"}};
        send_frame(helper, done);
        CHECK(helper.get_test_system_info().action == AmsAction::IDLE);
    }
}

TEST_CASE_METHOD(LVGLTestFixture, "ACE dryer commands name the unit on the fork",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    start_units(helper, client, 2, -1);
    helper.set_running(true);

    REQUIRE(helper.start_drying(50.0f, 60, -1, 1).success());
    REQUIRE(helper.stop_drying(1).success());
    REQUIRE(helper.stop_drying(0).success());

    CHECK(helper.captured_gcodes == std::vector<std::string>{
                                        "ACE_START_DRYING INSTANCE=1 TEMP=50 DURATION=60",
                                        "ACE_STOP_DRYING INSTANCE=1",
                                        "ACE_STOP_DRYING INSTANCE=0",
                                    });
}

// ACEPRO numbers its tools from a configurable base: with tool_base 4 the
// first unit carries T4-T7. Slots stay where they are; the driver's tool
// numbers are read off the slots, not assumed to equal the slot.
TEST_CASE_METHOD(LVGLTestFixture, "ACE maps the driver's tool numbers to slots",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    helper.test_on_started();
    REQUIRE_FALSE(client.rpc_calls.empty());
    json response = make_kobra_objects_query_response(6);
    for (int i = 0; i < 4; ++i) {
        response["result"]["status"]["ace_instance_0"]["slots"][i]["tool"] = 4 + i;
    }
    client.rpc_calls.back().success_cb(response);
    helix::ui::UpdateQueue::instance().drain();

    auto info = helper.get_test_system_info();
    CHECK(info.current_tool == 6);
    CHECK(info.current_slot == 2);
    CHECK(helper.get_slot_info(2).status == SlotStatus::LOADED);

    send_frame(helper, manager_delta({{"target_index", 5}}));
    CHECK(helper.get_test_system_info().pending_target_slot == 1);
    send_frame(helper, manager_delta({{"target_index", -1}}));

    helper.set_running(true);
    REQUIRE(helper.load_filament(3).success());
    REQUIRE_FALSE(helper.captured_gcodes.empty());
    CHECK(helper.captured_gcodes.back() == "ACE_CHANGE_TOOL TOOL=7");
}

// A unit is named after its model, as the fork's `protocol` states it; a
// second unit of the same model is numbered. The model string stays the
// unit's identity.
TEST_CASE_METHOD(LVGLTestFixture, "ACE units are named after their model",
                 "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    start_units(helper, client, 3, -1);

    const auto info = helper.get_test_system_info();
    REQUIRE(info.units.size() == 3);
    CHECK(info.units[0].display_name == "ACEPRO");
    CHECK(info.units[1].display_name == "ACEPRO2");
    CHECK(info.units[2].display_name == "ACEPRO2 #2");
    CHECK(info.units[0].name == "Anycubic Color Engine Pro");

    SECTION("an unknown protocol keeps the driver's model string") {
        json frame = json::object();
        frame["ace_instance_2"] = json{{"protocol", "ace9_future"}};
        send_frame(helper, frame);
        CHECK(helper.get_test_system_info().units[2].display_name.empty());
    }
}

// The largest rig ACEPRO is daisy-chained into: two ACE Pros and four ACE 2s.
// The bootstrap asks for every instance discovery found, so the fifth and
// sixth units are there from the start, not once their spools next change.
TEST_CASE_METHOD(LVGLTestFixture, "ACE shows six daisy-chained units", "[ams][ace][multiunit]") {
    helix::test::FakeMoonrakerClient client;
    AmsBackendAceTestHelper helper(&client);
    helper.set_discovery(ace_discovery(6));
    start_units(helper, client, 6, 17, /*ace_pro_units=*/2);

    const auto info = helper.get_test_system_info();
    REQUIRE(info.units.size() == 6);
    CHECK(info.total_slots == 24);
    CHECK(info.units[5].first_slot_global_index == 20);
    std::vector<std::string> names;
    for (const auto& unit : info.units) {
        names.push_back(unit.display_name);
    }
    CHECK(names == std::vector<std::string>{"ACEPRO", "ACEPRO #2", "ACEPRO2", "ACEPRO2 #2",
                                            "ACEPRO2 #3", "ACEPRO2 #4"});
    // T17 is the fifth unit's second slot.
    CHECK(info.current_slot == 17);
    CHECK(helper.get_slot_info(17).status == SlotStatus::LOADED);

    const ams_draw::SystemToolLayout layout = ams_draw::compute_system_tool_layout(info, &helper);
    CHECK(layout.total_physical_tools == 1);
    CHECK(layout.hub_groups.size() == 1);
}
