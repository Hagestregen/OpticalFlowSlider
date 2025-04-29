#!/usr/bin/env python3
import sqlite3
import os

# ——————————————————————————————————————————————
# CONFIGURE THESE:
BAG_PATH   = "/home/docker/OpticalFlowSlider/ros2_ws/my_rosbag/LFN3_notOOSM_KF_640/LFN3_notOOSM_KF_640_0.db3"
IMU_TOPIC  = "/inertialsense/imu"
FLOW_TOPIC = "/optical_flow/LFN3_velocity"
VERBOSE    = True  # set False to suppress per-event prints
# ——————————————————————————————————————————————

def check_oosm(bag_path, imu_topic, flow_topic, verbose=False):
    conn = sqlite3.connect(bag_path)
    c = conn.cursor()
    # get topic IDs
    c.execute("SELECT id, name FROM topics")
    topics = c.fetchall()
    topic_ids = {name: _id for _id, name in topics}
    if imu_topic not in topic_ids or flow_topic not in topic_ids:
        raise ValueError(f"Topics not found: {imu_topic}, {flow_topic}")
    imu_id, flow_id = topic_ids[imu_topic], topic_ids[flow_topic]

    # walk through IMU+flow messages in time order
    c.execute("""
        SELECT timestamp, topic_id 
          FROM messages 
         WHERE topic_id IN (?,?)
      ORDER BY timestamp ASC
    """, (imu_id, flow_id))

    last_imu_ts = -1
    total_flow  = 0
    oosm_count  = 0
    for ts, tid in c:
        if tid == imu_id:
            last_imu_ts = ts
        else:  # flow message
            total_flow += 1
            if last_imu_ts != -1 and ts < last_imu_ts:
                oosm_count += 1
                if verbose:
                    print(f"OOSM event: flow_ts {ts} < last_imu_ts {last_imu_ts}")

    conn.close()
    return total_flow, oosm_count

if __name__ == "__main__":
    if not os.path.isfile(BAG_PATH):
        print(f"ERROR: bag file not found: {BAG_PATH}")
        exit(1)

    total, oosm = check_oosm(BAG_PATH, IMU_TOPIC, FLOW_TOPIC, VERBOSE)
    print(f"Total flow messages: {total}")
    print(f"OOSM cases: {oosm}")
    if oosm > 0:
        print("=> OOSM detected! Enable out-of-sequence handling.")
    else:
        print("=> No OOSM detected; you can disable it.")
