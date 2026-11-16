"""Create a tiny SQLite state database for the real-input example."""

import sqlite3
from pathlib import Path

path = Path(__file__).with_name("state.sqlite")
connection = sqlite3.connect(path)
connection.executescript(
    """
    DROP TABLE IF EXISTS robots;
    DROP TABLE IF EXISTS tasks;
    DROP TABLE IF EXISTS resource_schedule;
    CREATE TABLE robots(id TEXT, robot_type TEXT, task_type TEXT, start_node TEXT, speed_xy REAL, speed_z REAL, available_at REAL);
    CREATE TABLE tasks(id TEXT, pickup TEXT, delivery TEXT, task_type TEXT, release_time REAL, status TEXT);
    CREATE TABLE resource_schedule(resource TEXT, robot_id TEXT, task_id TEXT, start REAL, end REAL, direction INTEGER);
    INSERT INTO robots VALUES ('r0', 'wheeled', 'pharmacy', 'f0_b', 1.5, 0.5, 0.0);
    INSERT INTO robots VALUES ('r1', 'wheeled', 'pharmacy', 'f0_a', 1.5, 0.5, 0.0);
    INSERT INTO tasks VALUES ('t0', 'f1_b', 'f1_a', NULL, 0.0, 'pending');
    INSERT INTO tasks VALUES ('t1', 'f1_a', 'f1_b', NULL, 0.0, 'pending');
    """
)
connection.commit()
connection.close()
print(path)
