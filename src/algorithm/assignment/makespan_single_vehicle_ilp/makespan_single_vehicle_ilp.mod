set REQUESTS;

param start_cost {REQUESTS} >= 0;
param transition_cost {REQUESTS cross REQUESTS} >= 0;
param end_cost {REQUESTS} >= 0;

var First_Request {REQUESTS} binary;
var Transition {REQUESTS cross REQUESTS} binary;
var Last_Request {REQUESTS} binary;

subject to Conservation {i in REQUESTS}:
  First_Request[i]
  + sum {j in REQUESTS} Transition[j, i]
  - sum {j in REQUESTS} Transition[i, j]
  - Last_Request[i] = 0;
subject to All_Incoming {j in REQUESTS}:
  First_Request[j] + sum {i in REQUESTS} Transition[i, j] = 1;
subject to All_Outgoing {i in REQUESTS}:
  sum {j in REQUESTS} Transition[i, j] + Last_Request[i] = 1;
subject to Single_First:
  sum {i in REQUESTS} First_Request[i] = 1;
subject to No_Same {i in REQUESTS}:
  Transition[i, i] = 0;

var Number {REQUESTS} >= 0;
param n = card(REQUESTS);
subject to Order {(i, j) in REQUESTS cross REQUESTS}:
  Number[j] >= Number[i] + 1 + (n + 1) * (Transition[i, j] - 1);

minimize Total_Distance:
  sum {(i, j) in REQUESTS cross REQUESTS} transition_cost[i, j] * Transition[i, j];
