/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if ENABLE_TELEMETRY_MAVLINK_MISSION

#include "common/time.h"

#include "drivers/time.h"

#include "config/config.h"

#include "fc/runtime_config.h"

#include "flight/flight_plan_nav.h"

#include "io/gps.h"

#include "pg/flight_plan.h"

#include "telemetry/mavlink.h"
#include "telemetry/mavlink_mission.h"

#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID MAV_COMP_ID_AUTOPILOT1

#define MISSION_UPLOAD_RETRY_MS   1500
#define MISSION_UPLOAD_MAX_RETRY  5
#define MISSION_DOWNLOAD_IDLE_MS  5000
#define MISSION_CURRENT_PERIOD_MS 1000

typedef enum {
    MISSION_IDLE,
    MISSION_RECEIVING,  // sent REQUEST_INT, awaiting MISSION_ITEM_INT
    MISSION_SENDING,    // sent COUNT, awaiting MISSION_REQUEST_INT
} missionState_e;

static struct {
    missionState_e state;
    uint16_t nextSeq;
    uint16_t totalCount;
    uint8_t  partnerSys;
    uint8_t  partnerComp;
    timeMs_t lastActivityMs;
    uint8_t  retries;
    timeMs_t lastCurrentTxMs;
    int16_t  pendingReachedIndex;
    bool     rtlTerminator;     // current upload ends with NAV_RETURN_TO_LAUNCH (last slot discarded)
} m;

static mavlink_message_t txMsg;

static bool targetIsUs(uint8_t target_system, uint8_t target_component)
{
    if (target_system && target_system != MAVLINK_SYSTEM_ID) {
        return false;
    }
    if (target_component && target_component != MAVLINK_COMPONENT_ID) {
        return false;
    }
    return true;
}

// During an active transfer, reject any traffic from a sysid/compid that isn't
// the partner we started the session with. Stops a second GCS from interleaving
// into an in-flight upload or download.
static bool senderIsActivePartner(const mavlink_message_t *msg)
{
    return msg->sysid == m.partnerSys && msg->compid == m.partnerComp;
}

static void sendAck(uint8_t partnerSys, uint8_t partnerComp, uint8_t type)
{
    mavlink_msg_mission_ack_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &txMsg,
        partnerSys, partnerComp, type, MAV_MISSION_TYPE_MISSION, 0);
    mavlinkSendMessage(&txMsg);
}

static void sendRequestInt(uint8_t partnerSys, uint8_t partnerComp, uint16_t seq)
{
    mavlink_msg_mission_request_int_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &txMsg,
        partnerSys, partnerComp, seq, MAV_MISSION_TYPE_MISSION);
    mavlinkSendMessage(&txMsg);
}

static void sendCount(uint8_t partnerSys, uint8_t partnerComp, uint16_t count)
{
    mavlink_msg_mission_count_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &txMsg,
        partnerSys, partnerComp, count, MAV_MISSION_TYPE_MISSION, 0);
    mavlinkSendMessage(&txMsg);
}

static void abortUpload(uint8_t result)
{
    sendAck(m.partnerSys, m.partnerComp, result);
    m.state = MISSION_IDLE;
}

static void sendMissionItem(uint8_t partnerSys, uint8_t partnerComp, uint16_t seq)
{
    const flightPlanConfig_t *plan = flightPlanConfig();
    const waypoint_t *wp = &plan->waypoints[seq];

    uint16_t command;
    float param1 = 0.0f;
    float param2 = 0.0f;
    int32_t lat = wp->latitude;
    int32_t lon = wp->longitude;
    float zM = wp->altitude * 0.01f;

    switch (wp->type) {
    case WAYPOINT_TYPE_HOLD:
        command = MAV_CMD_NAV_LOITER_TIME;
        param1 = wp->duration * 0.1f;
        break;
    case WAYPOINT_TYPE_LAND:
        command = MAV_CMD_NAV_LAND;
        break;
    case WAYPOINT_TYPE_TAKEOFF:
        command = MAV_CMD_NAV_TAKEOFF;
        break;
    case WAYPOINT_TYPE_ALT_CHANGE:
        command = MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT;
        param1 = (float)wp->pattern;     // climb-mode round-trip
        lat = 0; lon = 0;                // no horizontal target
        break;
    case WAYPOINT_TYPE_DELAY:
        command = MAV_CMD_NAV_DELAY;
        param1 = wp->duration * 0.1f;
        lat = 0; lon = 0; zM = 0.0f;
        break;
    case WAYPOINT_TYPE_YAW_RATE:
        command = MAV_CMD_NAV_SET_YAW_SPEED;
        param2 = (float)wp->speed;
        lat = 0; lon = 0; zM = 0.0f;
        break;
    case WAYPOINT_TYPE_FLYOVER:
    case WAYPOINT_TYPE_FLYBY:
    default:
        command = MAV_CMD_NAV_WAYPOINT;
        break;
    }

    const uint8_t current = (flightPlanNavIsActive() && seq == flightPlanNavGetCurrentIndex()) ? 1 : 0;

    mavlink_msg_mission_item_int_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &txMsg,
        partnerSys, partnerComp, seq, MAV_FRAME_GLOBAL_INT, command, current, 1,
        param1, param2, 0.0f, 0.0f,
        lat, lon, zM,
        MAV_MISSION_TYPE_MISSION);
    mavlinkSendMessage(&txMsg);
}

static bool decodeFrameAltCm(uint8_t frame, float z, int32_t *altCmOut, uint8_t *resultOut)
{
    if (!isfinite(z)) {
        *resultOut = MAV_MISSION_INVALID_PARAM7;
        return false;
    }
    switch (frame) {
    case MAV_FRAME_GLOBAL_INT:
        *altCmOut = (int32_t)(z * 100.0f);
        return true;
    case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
        if (!STATE(GPS_FIX_HOME)) {
            *resultOut = MAV_MISSION_INVALID;
            return false;
        }
        *altCmOut = (int32_t)(z * 100.0f) + GPS_home_llh.altCm;
        return true;
    default:
        *resultOut = MAV_MISSION_UNSUPPORTED_FRAME;
        return false;
    }
}

static bool mapMavCmdToWaypoint(const mavlink_mission_item_int_t *it, waypoint_t *wp,
                                bool isLastSlot, bool *isRtlTerminator, uint8_t *resultOut)
{
    *isRtlTerminator = false;

    if (it->command == MAV_CMD_NAV_RETURN_TO_LAUNCH) {
        if (!isLastSlot) {
            *resultOut = MAV_MISSION_UNSUPPORTED;
            return false;
        }
        *isRtlTerminator = true;
        return true;
    }

    // Modifier commands carry no horizontal position; the generic lat/lon range
    // check and frame-based alt decode are skipped (with ALT_CHANGE pulling alt
    // explicitly below). DELAY / SET_YAW_SPEED ignore all coord fields.
    const bool isModifier = (it->command == MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT)
                         || (it->command == MAV_CMD_NAV_DELAY)
                         || (it->command == MAV_CMD_NAV_SET_YAW_SPEED);

    int32_t altCm = 0;
    if (!isModifier) {
        if (!decodeFrameAltCm(it->frame, it->z, &altCm, resultOut)) {
            return false;
        }
        if (it->x < -900000000 || it->x > 900000000) {
            *resultOut = MAV_MISSION_INVALID_PARAM5_X;
            return false;
        }
        if (it->y < -1800000000 || it->y > 1800000000) {
            *resultOut = MAV_MISSION_INVALID_PARAM6_Y;
            return false;
        }
    }
    if (!isfinite(it->param1) || !isfinite(it->param2) || !isfinite(it->param3) || !isfinite(it->param4)) {
        *resultOut = MAV_MISSION_INVALID;
        return false;
    }

    wp->latitude = isModifier ? 0 : it->x;
    wp->longitude = isModifier ? 0 : it->y;
    wp->altitude = altCm;
    wp->speed = 0;
    wp->duration = 0;
    wp->pattern = WAYPOINT_PATTERN_ORBIT;

    switch (it->command) {
    case MAV_CMD_NAV_WAYPOINT:
    case MAV_CMD_NAV_SPLINE_WAYPOINT:
        // SPLINE_WAYPOINT shares the param1/x/y/z layout with NAV_WAYPOINT; the
        // executor flies a straight line through it, which is a faithful
        // approximation for the QGC plan-editor's curved preview.
        wp->type = WAYPOINT_TYPE_FLYBY;
        if (it->param1 > 0.0f) {
            const float ds = it->param1 * 10.0f;
            wp->duration = (ds >= UINT16_MAX) ? UINT16_MAX : (uint16_t)ds;
        }
        break;
    case MAV_CMD_NAV_LOITER_TIME:
        wp->type = WAYPOINT_TYPE_HOLD;
        if (it->param1 > 0.0f) {
            const float ds = it->param1 * 10.0f;
            wp->duration = (ds >= UINT16_MAX) ? UINT16_MAX : (uint16_t)ds;
        }
        break;
    case MAV_CMD_NAV_LOITER_TURNS:
        wp->type = WAYPOINT_TYPE_HOLD;
        break;
    case MAV_CMD_NAV_LOITER_UNLIM:
        wp->type = WAYPOINT_TYPE_HOLD;
        wp->duration = UINT16_MAX;
        break;
    case MAV_CMD_NAV_LOITER_TO_ALT:
        // Best-effort: hold at lat/lon. The "loiter until alt reached" latch
        // needs executor work — without it the waypoint advances on arrival
        // like any other HOLD-zero. Capability follow-up.
        wp->type = WAYPOINT_TYPE_HOLD;
        break;
    case MAV_CMD_NAV_FOLLOW:
        // Best-effort: hover at the initial target position. Dynamic vehicle
        // following needs an updating target source the executor doesn't have
        // today — capability follow-up.
        wp->type = WAYPOINT_TYPE_HOLD;
        break;
    case MAV_CMD_NAV_LAND:
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LAND_LOCAL:
        // VTOL_LAND is the multi-rotor-half of a quadplane landing; on a pure
        // multirotor it has the same semantics as NAV_LAND. LAND_LOCAL is the
        // local-frame variant — the frame check rejects actual LOCAL_NED items
        // before we get here; the case is kept for protocol completeness in
        // case a GCS sends LAND_LOCAL with a global frame.
        wp->type = WAYPOINT_TYPE_LAND;
        break;
    case MAV_CMD_NAV_PAYLOAD_PLACE:
        // Treated as LAND for now — descend to the named position. The actual
        // payload-release step is a capability follow-up (no release mechanism
        // on a Betaflight quad today).
        wp->type = WAYPOINT_TYPE_LAND;
        break;
    case MAV_CMD_NAV_TAKEOFF:
    case MAV_CMD_NAV_VTOL_TAKEOFF:
    case MAV_CMD_NAV_TAKEOFF_LOCAL:
        // VTOL_TAKEOFF likewise collapses to NAV_TAKEOFF on a multirotor; the
        // "transition heading" param is fixed-wing-only and ignored.
        // TAKEOFF_LOCAL is the local-frame variant — see LAND_LOCAL note.
        wp->type = WAYPOINT_TYPE_TAKEOFF;
        break;
    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT: {
        // Modifier: re-targets the next positional waypoint's altitude. param1
        // is the climb-mode (0=Neutral, 1=Climbing, 2=Descending) — preserved
        // for round-trip but not yet enforced by the executor.
        if (it->param1 < -0.5f || it->param1 > 2.5f) {
            *resultOut = MAV_MISSION_INVALID_PARAM1;
            return false;
        }
        int32_t modAltCm;
        if (!decodeFrameAltCm(it->frame, it->z, &modAltCm, resultOut)) {
            return false;
        }
        wp->type = WAYPOINT_TYPE_ALT_CHANGE;
        wp->altitude = modAltCm;
        wp->pattern = (uint8_t)lrintf(it->param1);
        break;
    }
    case MAV_CMD_NAV_DELAY: {
        // Modifier: scales cruise on the next leg so traversal ≈ delay seconds.
        // ToD form (param1 == -1 with HH:MM:SS) is rejected — Betaflight targets
        // don't universally have RTC.
        if (it->param1 < 0.0f) {
            *resultOut = MAV_MISSION_INVALID_PARAM1;
            return false;
        }
        const float ds = it->param1 * 10.0f;
        wp->type = WAYPOINT_TYPE_DELAY;
        wp->duration = (ds >= UINT16_MAX) ? UINT16_MAX : (uint16_t)ds;
        break;
    }
    case MAV_CMD_NAV_SET_YAW_SPEED:
        // Modifier: yaw-rate cap. Stored + round-tripped now; executor
        // consumption is gated on yaw control during AUTOPILOT_MODE landing,
        // which is a capability follow-up. param1 (angle to adjust) and
        // param3 (relative flag) are intentionally ignored.
        if (it->param2 < 0.0f) {
            *resultOut = MAV_MISSION_INVALID_PARAM2;
            return false;
        }
        wp->type = WAYPOINT_TYPE_YAW_RATE;
        wp->speed = (it->param2 >= (float)UINT16_MAX) ? UINT16_MAX : (uint16_t)it->param2;
        break;
    // Still rejected — no positional or modifier mapping that's safe to fit:
    //   NAV_ROI               — camera target; mapping to HOLD would fly the
    //                           vehicle to the camera point
    //   NAV_PATHPLANNING      — planner enable/disable, not a goal
    //   NAV_GUIDED_ENABLE     — off-board control toggle
    //   NAV_FENCE_* / NAV_RALLY_POINT — different mission_type, rejected
    //                           earlier by the MISSION_TYPE_MISSION filter
    default:
        *resultOut = MAV_MISSION_UNSUPPORTED;
        return false;
    }
    return true;
}

static void handleRequestList(const mavlink_message_t *msg)
{
    const uint16_t count = flightPlanConfig()->waypointCount;
    m.partnerSys = msg->sysid;
    m.partnerComp = msg->compid;

    if (count == 0) {
        sendCount(msg->sysid, msg->compid, 0);
        m.state = MISSION_IDLE;
        return;
    }

    m.state = MISSION_SENDING;
    m.nextSeq = 0;
    m.totalCount = count;
    m.lastActivityMs = millis();
    sendCount(msg->sysid, msg->compid, count);
}

static void handleRequestInt(const mavlink_message_t *msg)
{
    mavlink_mission_request_int_t req;
    mavlink_msg_mission_request_int_decode(msg, &req);

    if (!targetIsUs(req.target_system, req.target_component)) {
        return;
    }
    if (req.mission_type != MAV_MISSION_TYPE_MISSION) {
        sendAck(msg->sysid, msg->compid, MAV_MISSION_UNSUPPORTED);
        return;
    }
    if (m.state != MISSION_SENDING || !senderIsActivePartner(msg)) {
        return;
    }
    // Allow current seq or one-back retransmit; everything else is out of sequence.
    if (req.seq != m.nextSeq && !(m.nextSeq > 0 && req.seq == m.nextSeq - 1)) {
        sendAck(msg->sysid, msg->compid, MAV_MISSION_INVALID_SEQUENCE);
        m.state = MISSION_IDLE;
        return;
    }
    if (req.seq >= m.totalCount) {
        sendAck(msg->sysid, msg->compid, MAV_MISSION_INVALID_SEQUENCE);
        m.state = MISSION_IDLE;
        return;
    }

    sendMissionItem(msg->sysid, msg->compid, req.seq);
    if (req.seq == m.totalCount - 1) {
        // Final item — partner is expected to send MISSION_ACK; drop to IDLE on receipt
        // or via the SENDING-idle timeout.
        m.nextSeq = req.seq + 1;
    } else if (req.seq == m.nextSeq) {
        m.nextSeq++;
    }
    m.lastActivityMs = millis();
}

static void handleCount(const mavlink_message_t *msg)
{
    mavlink_mission_count_t mc;
    mavlink_msg_mission_count_decode(msg, &mc);

    if (!targetIsUs(mc.target_system, mc.target_component)) {
        return;
    }
    if (mc.mission_type != MAV_MISSION_TYPE_MISSION) {
        sendAck(msg->sysid, msg->compid, MAV_MISSION_UNSUPPORTED);
        return;
    }
    if (FLIGHT_MODE(AUTOPILOT_MODE)) {
        sendAck(msg->sysid, msg->compid, MAV_MISSION_DENIED);
        return;
    }
    if (mc.count > MAX_WAYPOINTS) {
        sendAck(msg->sysid, msg->compid, MAV_MISSION_NO_SPACE);
        return;
    }
    if (mc.count == 0) {
        flightPlanConfigMutable()->waypointCount = 0;
        saveConfigAndNotify();
        sendAck(msg->sysid, msg->compid, MAV_MISSION_ACCEPTED);
        m.state = MISSION_IDLE;
        return;
    }

    // MAVLink semantics: a new upload replaces any existing mission. Zero the
    // count immediately so any consumer (CLI dump, executor on next engage)
    // doesn't see a half-overwritten waypoint table mid-transfer.
    flightPlanConfigMutable()->waypointCount = 0;

    m.state = MISSION_RECEIVING;
    m.nextSeq = 0;
    m.totalCount = mc.count;
    m.partnerSys = msg->sysid;
    m.partnerComp = msg->compid;
    m.lastActivityMs = millis();
    m.retries = 0;
    m.rtlTerminator = false;
    sendRequestInt(msg->sysid, msg->compid, 0);
}

static void handleItemInt(const mavlink_message_t *msg)
{
    mavlink_mission_item_int_t it;
    mavlink_msg_mission_item_int_decode(msg, &it);

    if (!targetIsUs(it.target_system, it.target_component)) {
        return;
    }
    if (m.state != MISSION_RECEIVING || !senderIsActivePartner(msg)) {
        return;
    }
    if (it.mission_type != MAV_MISSION_TYPE_MISSION) {
        abortUpload(MAV_MISSION_UNSUPPORTED);
        return;
    }
    if (it.seq != m.nextSeq) {
        abortUpload(MAV_MISSION_INVALID_SEQUENCE);
        return;
    }

    waypoint_t wp;
    bool isRtl = false;
    const bool isLastSlot = (it.seq == m.totalCount - 1);
    uint8_t result = MAV_MISSION_ACCEPTED;
    if (!mapMavCmdToWaypoint(&it, &wp, isLastSlot, &isRtl, &result)) {
        abortUpload(result);
        return;
    }

    if (isRtl) {
        // RTL discards its own slot; mission count finalises at items received so far.
        m.rtlTerminator = true;
    } else {
        flightPlanConfigMutable()->waypoints[it.seq] = wp;
    }

    m.nextSeq++;
    m.lastActivityMs = millis();
    m.retries = 0;

    if (m.nextSeq < m.totalCount) {
        sendRequestInt(msg->sysid, msg->compid, m.nextSeq);
        return;
    }

    // Upload complete.
    const uint16_t finalCount = m.rtlTerminator ? m.totalCount - 1 : m.totalCount;
    flightPlanConfigMutable()->waypointCount = finalCount;
    saveConfigAndNotify();
    sendAck(msg->sysid, msg->compid, MAV_MISSION_ACCEPTED);
    m.state = MISSION_IDLE;
}

static void handleClearAll(const mavlink_message_t *msg)
{
    mavlink_mission_clear_all_t mc;
    mavlink_msg_mission_clear_all_decode(msg, &mc);
    if (!targetIsUs(mc.target_system, mc.target_component)) {
        return;
    }
    if (mc.mission_type != MAV_MISSION_TYPE_MISSION) {
        sendAck(msg->sysid, msg->compid, MAV_MISSION_UNSUPPORTED);
        return;
    }
    if (FLIGHT_MODE(AUTOPILOT_MODE)) {
        sendAck(msg->sysid, msg->compid, MAV_MISSION_DENIED);
        return;
    }
    flightPlanConfigMutable()->waypointCount = 0;
    saveConfigAndNotify();
    sendAck(msg->sysid, msg->compid, MAV_MISSION_ACCEPTED);
    m.state = MISSION_IDLE;
}

static void handleMissionAck(const mavlink_message_t *msg)
{
    mavlink_mission_ack_t ack;
    mavlink_msg_mission_ack_decode(msg, &ack);
    if (!targetIsUs(ack.target_system, ack.target_component)) {
        return;
    }
    if (m.state == MISSION_SENDING && senderIsActivePartner(msg)) {
        m.state = MISSION_IDLE;
    }
}

static void onWaypointReached(uint8_t index)
{
    m.pendingReachedIndex = index;
}

void mavMissionInit(void)
{
    memset(&m, 0, sizeof(m));
    m.state = MISSION_IDLE;
    m.pendingReachedIndex = -1;
    flightPlanNavSetReachedListener(onWaypointReached);
}

bool mavMissionHandleMessage(const mavlink_message_t *msg)
{
    // Each handler decodes the typed payload and applies its own target filter
    // via targetIsUs(); per-message session-partner enforcement also lives in
    // the handlers that act on an active transfer.
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
        mavlink_mission_request_list_t r;
        mavlink_msg_mission_request_list_decode(msg, &r);
        if (!targetIsUs(r.target_system, r.target_component)) {
            return true;
        }
        if (r.mission_type != MAV_MISSION_TYPE_MISSION) {
            sendAck(msg->sysid, msg->compid, MAV_MISSION_UNSUPPORTED);
            return true;
        }
        handleRequestList(msg);
        return true;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        handleRequestInt(msg);
        return true;
    case MAVLINK_MSG_ID_MISSION_COUNT:
        handleCount(msg);
        return true;
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        handleItemInt(msg);
        return true;
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        handleClearAll(msg);
        return true;
    case MAVLINK_MSG_ID_MISSION_ACK:
        handleMissionAck(msg);
        return true;
    default:
        return false;
    }
}

void mavMissionUpdate(timeMs_t nowMs)
{
    // RECEIVING timeout: retransmit REQUEST_INT, give up after MAX_RETRY.
    if (m.state == MISSION_RECEIVING && (nowMs - m.lastActivityMs) >= MISSION_UPLOAD_RETRY_MS) {
        if (m.retries >= MISSION_UPLOAD_MAX_RETRY) {
            abortUpload(MAV_MISSION_OPERATION_CANCELLED);
        } else {
            m.retries++;
            m.lastActivityMs = nowMs;
            sendRequestInt(m.partnerSys, m.partnerComp, m.nextSeq);
        }
    }

    // SENDING idle: spec leaves recovery to the partner; just drop state.
    if (m.state == MISSION_SENDING && (nowMs - m.lastActivityMs) >= MISSION_DOWNLOAD_IDLE_MS) {
        m.state = MISSION_IDLE;
    }

    // MISSION_ITEM_REACHED — drain as soon as the executor flagged one.
    if (m.pendingReachedIndex >= 0) {
        mavlink_msg_mission_item_reached_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &txMsg,
            (uint16_t)m.pendingReachedIndex);
        mavlinkSendMessage(&txMsg);
        m.pendingReachedIndex = -1;
    }

    // MISSION_CURRENT @ 1 Hz — lets QGC display the live cursor whether or not
    // the mission is being executed.
    if ((nowMs - m.lastCurrentTxMs) >= MISSION_CURRENT_PERIOD_MS) {
        const uint16_t total = flightPlanConfig()->waypointCount;
        const bool active = flightPlanNavIsActive();
        const uint16_t seq = (total && active) ? flightPlanNavGetCurrentIndex() : 0;
        const uint8_t missionMode = active ? 1 : 2;
        uint8_t missionState;
        if (total == 0) {
            missionState = MISSION_STATE_NO_MISSION;
        } else if (!active) {
            missionState = MISSION_STATE_NOT_STARTED;
        } else if (flightPlanNavGetState() == FP_NAV_COMPLETE) {
            missionState = MISSION_STATE_COMPLETE;
        } else {
            missionState = MISSION_STATE_ACTIVE;
        }
        mavlink_msg_mission_current_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &txMsg,
            seq, total, missionState, missionMode, 0, 0, 0);
        mavlinkSendMessage(&txMsg);
        m.lastCurrentTxMs = nowMs;
    }
}

#endif // ENABLE_TELEMETRY_MAVLINK_MISSION
