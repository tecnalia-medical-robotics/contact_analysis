contact_cop
====================

# General description of the package
Analysis of the Center of Pressure related to contact points
<!--- protected region package descripion begin -->
<!--- protected region package descripion end -->

<!--- todo How to handle the image generation -->
<!--- <img src="./model/wrench_from_csv.png" width="300px" />-->

# Node: contact_cop_study
Update frequency: 1000 Hz.

<!--- protected region contact_cop_study begin -->
<!--- protected region contact_cop_study end -->

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

## Published Topics
`wrench` *(geometry_msgs::WrenchStamped)*
<!--- protected region wrench begin -->
<!--- protected region wrench end -->

