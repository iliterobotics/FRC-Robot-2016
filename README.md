# THIS IS THE 2015 ROBOT CODE DEVELOPMENT BRANCH
#

#How to setup the enviroment to push to RoboRIO

1. First install the FRC Update suite, located at:
http://www.ni.com/download/first-robotics-software-2015/5112/en/

This is needed because the RoboRio uses mDns to connect. 
By default Windows does not include an install. The FRC Update
Suite includes an implementation of the mDns protocol handler.

2. Setup Eclipse
Follow these instructions to install Eclipse and get the necessary
plug ins and libraries: 
http://wpilib.screenstepslive.com/s/4485/m/13809/l/145002-installing-eclipse-c-java


#Other Important Resources
Other important links: 
-RoboRIO Networking: http://wpilib.screenstepslive.com/s/4485/m/13503/l/242608-roborio-networking

#How to connect to RoboRIO with webview
To connect to the webview, use: http://roborio-1885.local This has to be run on a non-chrome browser
because of Microsoft Silverlight

After installing open the build.properties located at: user.dir/wpilib/java/current/ant