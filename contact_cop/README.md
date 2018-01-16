contact_cop
====================

# General description of the package
Analysis of the Center of Pressure related to contact points
<!--- protected region package descripion begin -->

To launch the csv interpretor:
```
rosrun contact_cop wrench_from_csv _csv_file:=/home/anthony/tmp/sarafun_data/ibd_v2/insertion_by_deformation_annotated_data/data01_split/wrench_0.csv _inc:=5
```

<!--- protected region package descripion end -->

<!--- todo How to handle the image generation -->
<!--- <img src="./model/wrench_from_csv.png" width="300px" />-->

# Node: contact_cop_study
Update frequency: 100 Hz.

<!--- protected region contact_cop_study begin -->
<!--- protected region contact_cop_study end -->

## Dynamic Parameters
All dynamic parameters can be set through the command line:
```
rosrun contact_cop contact_cop_study _[param_name]:=[new_value]
```
`force_th` *(double, default: 1.0)*
<!--- protected region force_th begin -->
<!--- protected region force_th end -->

## Published Topics
`cop` *(geometry_msgs::Point)*
<!--- protected region cop begin -->
<!--- protected region cop end -->
`marker_cop` *(visualization_msgs::MarkerArray)*
<!--- protected region marker_cop begin -->
<!--- protected region marker_cop end -->

## Subscribed Topics
`wrench` *(geometry_msgs::WrenchStamped)*
<!--- protected region wrench begin -->
<!--- protected region wrench end -->

# Node: wrench_from_csv
Update frequency: 1000 Hz.

<!--- protected region wrench_from_csv begin -->
<!--- protected region wrench_from_csv end -->

## Dynamic Parameters
All dynamic parameters can be set through the command line:
```
rosrun contact_cop wrench_from_csv _[param_name]:=[new_value]
```
`csv_file` *(std::string, default: "Undef")*
<!--- protected region csv_file begin -->
<!--- protected region csv_file end -->
`inc` *(int, default: 1)*
<!--- protected region inc begin -->
<!--- protected region inc end -->

## Published Topics
`wrench` *(geometry_msgs::WrenchStamped)*
<!--- protected region wrench begin -->
<!--- protected region wrench end -->

