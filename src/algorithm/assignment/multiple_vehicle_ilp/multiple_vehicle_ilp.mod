set REQUESTS;
set ROBOTS;

param start_cost {ROBOTS cross REQUESTS} >= 0;
param transition_cost {REQUESTS cross REQUESTS} >= 0;
set END_POSITIONS = ROBOTS;
param end_cost {REQUESTS cross END_POSITIONS};

var First_Request {ROBOTS cross REQUESTS} binary;
var Transition {ROBOTS cross REQUESTS cross REQUESTS} binary;
var Last_Request {ROBOTS cross REQUESTS cross END_POSITIONS} binary;

subject to Conservation {r in ROBOTS, i in REQUESTS}:
  First_Request[r, i]
  + sum {j in REQUESTS} Transition[r, j, i]
  - sum {j in REQUESTS} Transition[r, i, j]
  - sum {e in END_POSITIONS} Last_Request[r, i, e] = 0;
subject to All_Incoming {j in REQUESTS}:
  sum {r in ROBOTS} (First_Request[r, j] + sum {i in REQUESTS} Transition[r, i, j]) = 1;
subject to Max_Single_First {r in ROBOTS}:
  sum {i in REQUESTS} First_Request[r, i] <= 1;
subject to No_Same {r in ROBOTS, i in REQUESTS}:
  Transition[r, i, i] = 0;

subject to All_Other_End_Position {e in END_POSITIONS}:
  sum {r in ROBOTS, j in REQUESTS} Last_Request[r, j, e] = 1;

var Robot_Total {ROBOTS};
subject to Calculate_Robot_Total {r in ROBOTS}:
  sum {i in REQUESTS} start_cost[r, i] * First_Request[r, i]
  + sum {(i, j) in REQUESTS cross REQUESTS} transition_cost[i, j] * Transition[r, i, j]
  + sum {i in REQUESTS, e in END_POSITIONS} end_cost[i, e] * Last_Request[r, i, e] = Robot_Total[r];

var Robot_Nr_Requests {ROBOTS} integer;
subject to Calculate_Robot_Nr_Requests {r in ROBOTS}:
  Robot_Nr_Requests[r] = sum {(i, j) in REQUESTS cross REQUESTS} Transition[r, i, j] + 1;

var Robot_Number {ROBOTS cross REQUESTS} >= 0 integer;
subject to Order {r in ROBOTS, (i, j) in REQUESTS cross REQUESTS}:
  Robot_Number[r, j] >= Robot_Number[r, i] + 1 + (Robot_Nr_Requests[r] + 1) * (Transition[r, i, j] - 1);
subject to Maximum_Number {r in ROBOTS, i in REQUESTS}:
  Robot_Number[r, i] <= (Robot_Nr_Requests[r] - 1) * (First_Request[r, i] + sum {j in REQUESTS} Transition[r, j, i]);

var Maximum;
subject to Overall_Maximum {r in ROBOTS}:
  Maximum >= Robot_Total[r];

minimize Total_Duration:
  Maximum;
