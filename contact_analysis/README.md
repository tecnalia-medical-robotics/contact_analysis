contact_analysis
====================

# General description of the package
Analysis of the Center of Pressure related to contact points
<!--- protected region package descripion begin -->
<!--- protected region package descripion end -->

<!--- todo How to handle the image generation -->
<!--- <img src="./model/contact_evaluate.png" width="300px" />-->

# Node: contact_cop
Update frequency: 1000 Hz.

<!--- protected region contact_cop begin -->
<!--- protected region contact_cop end -->

## Dynamic Parameters

All dynamic parameters can be set through the command line:
```
rosrun contact_analysis contact_cop _[param_name]:=[new_value]
```
`force_th` *(double, default: 1.0)*
<!--- protected region param force_th begin -->
minmum force norm to compute COP
<!--- protected region param force_th end -->

## Published Topics

A topic can be remapped from the command line:
```
rosrun contact_analysis contact_cop [old_name]:=[new_name]
```

`cop` *(geometry_msgs::Point)*
<!--- protected region publisher cop begin -->
Computed Center of Pressure
<!--- protected region publisher cop end -->
`marker_cop` *(visualization_msgs::MarkerArray)*
<!--- protected region publisher marker_cop begin -->
Marker for rviz at the computed COP
<!--- protected region publisher marker_cop end -->

## Subscribed Topics

A topic can be remapped from the command line:
```
rosrun contact_analysis contact_cop [old_name]:=[new_name]
```

`wrench` *(geometry_msgs::WrenchStamped)*
<!--- protected region subscriber wrench begin -->
Wrench measured by a sensor
<!--- protected region subscriber wrench end -->

# Node: wrench_from_csv
Update frequency: 1000 Hz.

<!--- protected region wrench_from_csv begin -->
<!--- protected region wrench_from_csv end -->

## Dynamic Parameters

All dynamic parameters can be set through the command line:
```
rosrun contact_analysis wrench_from_csv _[param_name]:=[new_value]
```
`csv_file` *(std::string, default: "Undef")*
<!--- protected region param csv_file begin -->
csv file containing wrench information to publish
<!--- protected region param csv_file end -->
`inc` *(int, default: 1)*
<!--- protected region param inc begin -->
from the wrench measure, increment applied at each publication loop
<!--- protected region param inc end -->
`slice_file` *(std::string, default: "Undef")*
<!--- protected region param slice_file begin -->
file containing a list of relevant slices
<!--- protected region param slice_file end -->
`label_file` *(std::string, default: "Undef")*
<!--- protected region param label_file begin -->
event name (see slice file structure)
<!--- protected region param label_file end -->

## Published Topics

A topic can be remapped from the command line:
```
rosrun contact_analysis wrench_from_csv [old_name]:=[new_name]
```

`wrench` *(geometry_msgs::WrenchStamped)*
<!--- protected region publisher wrench begin -->
emulated wrench measure
<!--- protected region publisher wrench end -->
`data_info` *(visualization_msgs::MarkerArray)*
<!--- protected region publisher data_info begin -->
RVIZ Marker for displaying text advancement information
<!--- protected region publisher data_info end -->

# Node: contact_evaluate
Update frequency: 200 Hz.

<!--- protected region contact_evaluate begin -->
<!--- protected region contact_evaluate end -->

## Subscribed Topics

A topic can be remapped from the command line:
```
rosrun contact_analysis contact_evaluate [old_name]:=[new_name]
```

`cop` *(geometry_msgs::Point)*
<!--- protected region subscriber cop begin -->
computed cop
<!--- protected region subscriber cop end -->

## Action proposed

A simple action launched can be obtained with:

```
rosrun actionlib axclient.py /do_action
```

Any action name can be readjusted at node launch:

```
rosrun contact_analysis contact_evaluate _[old_name]:=[new_name]
```

`learn` *(contact_msgs::LearnContact)*
<!--- protected region action server learn begin -->
to learn a new contact
<!--- protected region action server learn end -->
`evaluate` *(contact_msgs::EvaluateContact)*
<!--- protected region action server evaluate begin -->
to evaluate a contact taking place
<!--- protected region action server evaluate end -->
