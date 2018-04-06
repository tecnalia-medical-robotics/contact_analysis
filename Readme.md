# SARAFun contact analysis

This code is developed in the context of SARAFun.

## Content
* Package `contact_analysis` : main package to analyze a contact based on Center of Pressure (COP):
 * Node `contact_cop`: compute the COP from a stamped wrench received by subscription
 * Node `contact_evaluate`: provides two actions, for learning and, later on, evaluating a contact taking place.
 * Node `wrench_from_csv`: publish a wrench from a csv file containing only wrench information (examples in [contact_test_data/data/wrench0_sample.csv](contact_test_data/data/wrench0_sample.csv) )
* Package `contact_def` :  pure python package containing a contact description as well as a set of contacts handler.
* Package `contact_msgs`: contains all actions, services and specific messages created.
* Package `contact_plot`: contains a program to display learned contacts, as well as contact to be evaluated.
* Package `contact_test_data`: contains a set of wrench sample that can be used for testing of the component.

## Use

The following example is provided considering an Optoforce sensor, together with the ROS driver available [there](https://github.com/tecnalia-medical-robotics/optoforce_ros).
Note that any other ROS force sensor could be used, as long as it publishes a [WrenchStamped][wrench_stamped_url] message.

[wrench_stamped_url]: http://docs.ros.org/api/geometry_msgs/html/msg/WrenchStamped.html

### Basic Launch
```
roscore
# launch your force sensor node
roslaunch optoforce_ros optoforce_node.launch
# Let us assume the wrenchStmped is published on /optoforce_node/wrench_IRE0A005
# To generate the COP (Center of Pressure) used in the evaluation process
rosrun contact_analysis contact_cop wrench:=/optoforce_node/wrench_IRE0A005
# To enable contact learning / evaluation
rosrun contact_analysis contact_evaluate
# To display the COP live
rosrun contact_plot contact_plot.py
```

### Loading a learned set of contacts
If available, a set of contacts previously learned can be provided to contact evaluator (`contact_evaluate`) and the COP display (`contact_plot`):

```
# for the evaluator
rosservice call /load "message: '/tmp/phone_labelled/config.yaml'"
# for the plot
rosservice call /plot_load "message: '/tmp/phone_labelled/config.yaml'"
```
For the plot, the figure needs to be closed after the call to load the set of contacts as defined in the yaml file

### Storing a set of contacts
Once a set of contacts have been demonstrated, they can be stored for later use:
```
rosservice call /store "message: '/tmp/phone_labelled'"
```
This will write in the given directory:
* one file per contact, containing all its spec.
* a common file named `config.yaml` describing the list of contacts present in the repository.

One can open `config.yaml` and adjust the file to be later used.

__If the directory exists, there is no check on its content: previous content is likely to be overwritten!__

### Learning a contact
We assume the contact is done, and thus sensed by the force sensor.

This is done calling the action `learn` of node `contact_evaluate`.
To learn, follow the [LearnContact.action][learn_action] format:
```
# action called for learning a new contact during a given period
# frequency at which the cop should be looked for
# if unset or 0, default config value used
int32 frequency
# duration (s) of the recording
# if unset or 0, default config value used
float32 learning_duration
# whether the current contact is considered to be good or bad
bool is_good_contact
# text label of the current contact
string contact_label
---
# result definition (likely to be extended)
bool success
---
# feedback
# number of sample currently accumulated
int32 sample_number
```
We recommend not touching the frequency.
For the recording duration, 2 to 3 seconds should be enough.

[learn_action]: contact_msgs/action/LearnContact.action

### Evaluating a contact

We assume the contact is done, and thus sensed by the force sensor.

This is done calling the action `evaluate` of node `contact_evaluate`.
To learn, follow the [EvaluateContact.action][evaluate_action] format:
```
# action called to trigger the evaluation of the current contact
# frequency at which the cop should be looked for
# if unset or 0, default config value used
int32 frequency
# duration (s) of the recording
# if unset or 0, default config value used
float32 learning_duration
---
# result definition (likely to be extended)
bool success
# most probable contact (good vs bad)
bool is_good
# confidence (0 to 1)
float32 confidence
# label of the most probable blob
string blob_label
# id of the most probable blob
int32 blob_id
# additional message
string message
---
# feedback
# number of sample currently accumulated
int32 sample_number
```
Again, better to leave the frequency at the default value.
The `learning_duration` can be set to 2 to 3 seconds.

The `confidence` flag indicate how much we are confident in the decision we make.
1 means we are pretty confident.
Lower values means the evaluation may be erroneous.

[evaluate_action]: contact_msgs/action/EvaluateContact.action

### Tools
The action can be triggered directly using `actionlib` package:
```
rosrun actionlib axclient.py /learn
rosrun actionlib axclient.py /evaluate

```

When `contact_plot` is loaded, any new contact set (through learning or evaluation) is transmitted by `contact_evaluation` and displayed in the plot.
The following may be deprecated:
* the launch file contained in `contact_test_data`



**Warning**
The current code is under development.
The COP evaluation is for instance just the skeleton.
No intelligence placed in it so far.

**Checking the good installation**

You can simply run the following code, once the workspace built:

```
roslaunch contact_test_data skeleton_fake_data.launch
```
This will (1) load a wrench sample file, and publish it (`wrench_from_csv`), (2) compute the related COP (`contact_cop`), and provide actions for learning a contact model and evaluating it (`contact_evaluate`), and finally launch a action gui client to launch one of the two actions (through `axclient.py`)

**Connecting to a real force sensor**

```
roslaunch contact_analysis skeleton.launch force_topic:=[topic_name]
```
where `[topic_name]` is to be adjusted according to the real setup.
