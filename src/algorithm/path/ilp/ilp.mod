param T > 0 integer;
set TIMES = 0..T;

set ROBOTS;
set LOCATIONS;
set EDGES within LOCATIONS cross LOCATIONS;

set TIMES_ROBOTS_LOCATIONS {TIMES cross LOCATIONS} within LOCATIONS;
param cost {r in ROBOTS, TIMES_ROBOTS_LOCATIONS[T, r]};

var Time_Robot_Location {(t, r) in TIMES cross ROBOTS, TIMES_ROBOTS_LOCATIONS[t, r]} binary;
subject to Robot_One_Location {r in ROBOTS, t in TIMES}:
  sum {i in TIMES_ROBOTS_LOCATIONS[t, r]} Time_Robot_Location[t, r, i] = 1;
subject to Movement {t in 0..(T - 1), r in ROBOTS, i in TIMES_ROBOTS_LOCATIONS[t, r]}:
  Time_Robot_Location[t, r, i] <=
  sum {(i, j) in EDGES: j in TIMES_ROBOTS_LOCATIONS[t + 1, r]} Time_Robot_Location[t + 1, r, j];
subject to Not_Same_Location {t in TIMES, i in LOCATIONS}:
  sum {r in ROBOTS: i in TIMES_ROBOTS_LOCATIONS[t, r]} Time_Robot_Location[t, r, i] <= 1;
subject to Move_To_Empty_Location {t in 0..(T - 1), r in ROBOTS, i in TIMES_ROBOTS_LOCATIONS[t, r]}:
  Time_Robot_Location[t, r, i] + sum {s in ROBOTS: s <> r && i in TIMES_ROBOTS_LOCATIONS[t + 1, s]} Time_Robot_Location[t + 1, s, i] <= 1;

minimize Total_Cost:
  sum {r in ROBOTS, t in TIMES} sum {i in TIMES_ROBOTS_LOCATIONS[t, r]} t * cost[r, i] * Time_Robot_Location[T, r, i];
