Launch files for gazebo, some model files and nodes for the assessment.

To get the best results it is important to update the version of gazebo installed.

Ensure you virtual machines ethernet connection is active, and you computer has an internet connection and then run these commands one after the other in a terminal window.


sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt-get install gazebo7 -y


If you have issues with any of the turtlebot3 examples then clone the turtlebot3 packages into your workspace and build them along with your other packages.