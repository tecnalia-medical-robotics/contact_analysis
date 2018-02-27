# SARAFun contact analysis

This code is developed in the context of SARAFun.

## Content

* `contact_analysis` : main nodes to analyse a contact based on Center of Pressure (COP):
 * `contact_cop`: compute the COP from a stamped wrench recieved by subscription
 * `contact_evaluate`: provides two actions, for learning and, later on, evaluating a contact takin place.
 * `wrench_from_csv`: publish a wrench from a csv file containing only wrench information (examples in [contact_test_data/data/wrench0_sample.csv](contact_test_data/data/wrench0_sample.csv) )
* `contact_test_data`: contains a set of wrench sample that can be used for testing of the component.

## Use

**Warning**
The current code is under development.
The COP evaluation is for instance just the skeleton.
No intelligence placed in it so far.

**Checking the good installation**

You can simply run the following code, once the workspace built:

```
roslaunch contact_test_data skeleton_fake_data.launch
```
This will (1) laod a wrench sample file, and publish it (`wrench_from_csv`), (2) compute the related COP (`contact_cop`), and provide actions for learning a contact model and evaluating it (`contact_evaluate`), and finally launch a action gui client to launch one of the two actions (through `axclient.py`)

**Connecting to a real force sensor**

```
roslaunch contact_analysis skeleton.launch force_topic:=[topic_name]
```
where `[topic_name]` is to be adjusted according to the real setup.
