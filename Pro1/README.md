# Robot
You should run under webots
# Introduction
It is the year 2020, Autonomous Vehicles (AV) are now the main means of transportation in the US. On
December 1st, 2020, you are at work in downtown Phoenix. From your corporate office in the
corporation’s skyscraper, you observe that the mayor is installing over every intersection in the downtown
large decorative balls of different colors for the festivities. The video walls outside your office show news
from the stock market. Suddenly, all the video walls show a “no signal” message. You hear from the local
radio that a powerful solar storm1 has hit the earth destroying all the satellites in orbit. All the satellite
telecommunications and, most importantly, the GPS satellites are dead. The existing AV technology was
not prepared to operate without GPS technology. Also, AV manufacturers were so proud of their systems,
they did not install steering wheels, gas and brake pedals on the AVs. People are in panic and try to flee
thinking that the worst is yet to come. Since they cannot use AV (owned or on demand), they try to find
their old cars. Unfortunately, since most people have already forgotten how to drive2
, there are many
accidents and abandoned cars on the highways and on the roads. On the other hand, the downtown is
virtually empty.
After dropping you off at work, your personal AV was out for chores and then it was trying to find a parking
spot when the GPS failed. This means that right now your car is lost – it does not know where it is or how
to find you. Luckily, you took the Introduction to Robotics class back in 2018 and you have built your own
AV software during the winter break that year. Unfortunately, you also followed what AV companies did,
and you didn’t install steering wheel and pedals on your AV. However, you have access to all software on
your car and you can do an over-the-air software update by using the local cellular networks which are
still operational. You have the brilliant idea to use the New Year’s decorations for localization. You are in
the mayor’s committee for the festivities and you have a draft map of the possible colors to be used at
each intersection. Your first goal is to localize the car in the downtown so that it can pick you up and then
pick up your kids from the school. Finally, you plan to leave the city and go to the countryside where you
expect that your family will be safer than the city.
Your vehicle has a LIDAR on the top and a camera facing forward, right above the windshield (See Figure
7). You may or may not want to use all the LIDAR point cloud (it is a large amount of data).
# Milestone one: Driving on the highway
Your vehicle will be placed on the beginning of the highway (see Figure 5 The curved road is the 3-lane
highway.), on one of the lanes, with the correct initial orientation. Your goal is to reach your home
(location “goal” in Figure 1 City Map with the intersection IDs) as fast as possible without crashing into
the stopped vehicles. Your car has a LIDAR and a camera that you could use. You do not have to stop at
the end of the road: you can jump off the speeding vehicle!
Instructions: You are not allowed to add sensors to the vehicle. You must only use the existing API for the
provided sensors. Your deliverable submitted as a zip file in Canvas should contain your “controllers”
folder with all your controller files. The controller name provided to you should not be changed so that
we can run or compile your code by only taking your controllers folder without modifying the rest of the
setup. If you choose to use Python, it has to be 64-bit Python 3.6 with standard packages (please contact
TA if you plan to use any python modules other than pandas for reading csv file and numpy). If you use a
compiled language, make sure that you provide everything needed so that it will compile on a Windows
10 64-bit computer with standard setup. It is recommended that you confirm that your code runs without
any dependencies by trying your code on a clean machine where you install Webots and download your
zip package from Canvas. Recall that the due date is right before the date that grades are due; hence, there
is not much time for revisions and re-grading.
# Milestone two: localization and driving in the city
Your car is placed somewhere randomly in the city. There is fog and only the color of the decorative ball
that is the closest is visible to the camera on your AV (See Figure 4 View from your AV. Because of the fog,
you can only see the color of the closest decorative ball.). Do not forget that the battery capacity on the
electric AV is limited and you have to go to a safe place as quickly as possible. The maximum time limit
you will have is 4 times the time taken by an ideal implementation that drives the car with 20 km/h. You
can drive faster than that and you can slow down at the intersections when you are making turns.
At each intersection, there are different possible colors preset for the decorative ball. The color of each
ball is selected every morning based on these probabilities. You have a csv file, which contains these
probabilities, but you do not know which colors were selected that day. Your controller should read a CSV
file named “colors.csv” which contains the following data for each intersection (one line per intersection):
Intersection_id, r1, g1, b1, p1, r2, g2, b2, p2, r3, g3, b3, p3
Intersection Id is an integer between 1 and 16. For each possible color, its RGB value is represented by ri,
gi, bi and possibility of selecting that color is represented by pi. In this specific example, r1 is the red, g1
is the green, b1 is the blue for the first possible color for the ball at that intersection and p1 is the
probability for the first color to be displayed. Similarly, the second possible color for the ball is represented
by r2, g2, b2 with the corresponding probability p2. Here, r, g, b and p are floating point numbers between
0.0 and 1.0.
Contents of an example csv file (which has three possible colors) is as follows:
intersection,r1,g1,b1,p1,r2,g2,b2,p2,r3,g3,b3,p3
1,1,0,0,0.8,1,1,0,0.2,1,0,1,0.0
2,1,0,0,0.3,1,1,0,0.6,1,0,1,0.1
3,1,0,0,0.2,1,1,0,0.1,1,0,1,0.7
4,1,0,0,0.8,1,1,0,0.2,1,0,1,0.0
5,1,0,0,0.3,1,1,0,0.6,1,0,1,0.1
6,1,0,0,0.2,1,1,0,0.1,1,0,1,0.7
7,1,0,0,0.8,1,1,0,0.2,1,0,1,0.0
8,1,0,0,0.3,1,1,0,0.6,1,0,1,0.1
9,1,0,0,0.3,1,1,0,0.6,1,0,1,0.1
10,1,0,0,0.3,1,1,0,0.6,1,0,1,0.1
11,1,0,0,0.3,1,1,0,0.6,1,0,1,0.1
12,1,0,0,0.3,1,1,0,0.6,1,0,1,0.1
13,1,0,0,0.8,1,1,0,0.2,1,0,1,0.0
14,1,0,0,0.3,1,1,0,0.6,1,0,1,0.1
15,1,0,0,0.2,1,1,0,0.1,1,0,1,0.7
16,1,0,0,0.3,1,1,0,0.6,1,0,1,0.1

Remark: You will have to abstract the environment of Figure 1 into a graph so that you can use Bayesian
reasoning. Your goal is to find out in which road segment you most likely are given some observations.
That is, you aim to answer the question what is the probability P(r | c) that you are in road segment
r=(i1,i2) given that you are observing color c at the intersection in front of you. You can drive randomly
until you are certain enough about where you are.



