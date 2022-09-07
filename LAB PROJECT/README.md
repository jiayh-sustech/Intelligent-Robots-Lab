# Project – Pokemon Turtle

## 1. Introduction

In this class, you have to complete a final task in the end of semester. You shall master the skill of coding ROS programs and using ROS tools for general projects. The final project will let you show your intelligence and passion in robot project, meanwhile it is very interesting! 

The name of this final project is called Pokemon Turlte in maze. You are asked to use turtlebot3 to grab Pokemons and walk out of the maze. In Researching Building I (because of the coronavirus, we may do it in Gazebo), we will provide a scene of a little maze constructed by sponge squares as shown in Figure 1. 

Somewhere in the maze hidden some Pokemons (some pokemon pictures posted upon the walls). You don’t know what the maze looks like, it may be dangerous (you should **stand away from the maze without seeing it** and you should not crash the wall). So you send your turtlebot to explore the maze by remote control or automatic control. Then grab some Pokemons (save some pokemon images in your computer under some rules).

![image-20220907174946546](https://i0.hdslb.com/bfs/album/26a82bbc6ff2dc4b6807aaa692af5519ca9d6dd4.png)

<center>Figure 1. The maze enviroment in Gazebo.</center>

![image-20220907174958647](https://i0.hdslb.com/bfs/album/9565d18a70c2e401f39579b8c1e29fc288e09e4f.png)

<center>Figure 2. The sponge maze in Researching Building Ⅰ.</center>

## 2. Rules

Because some of the wild Pokemons are very cautious and timid. While the others are fierce. It is not easy to catch a Pokemon. However, Professor Haokido gave some instructions that you should keep a certain distance and angles to catch a Pokemon. He gave us a target device shown as Figure 3. If we can keep a Pokemon in the center of the red rectangle and fit the size well, it will be easily caught.

![image-20220907174819987](https://i0.hdslb.com/bfs/album/ecccaf2da58695586f2f81b522066e301e0ce174.png)

<center>Figure 3. Catching target.</center>

Here are the rules: 

- Six Pokemons are qualified but you can get as many as you can. 

- <font color=#0000FF>A valid catching is shown in **Figure 4** .The red box is the picture, and the green box is the pokemon. Since pokemon is in the picture, the green box must be in the red box, and if you encounter two boxes whose coordinates are almost the same, you can enlarge the red box a little bit.</font>

<center><font color=#FF0000 size=4>Remember a valid catching as follows:</font></center>

![image-20220906235506661](https://i0.hdslb.com/bfs/album/dc9d12c243edd65f286cfbcd46a190f8c129d527.png)

<center>Figure 4. <font color=#0000FF>A valid catching</font>.</center>

- Your score depends on the following things: 

| Technique                             | Points                         |
| ------------------------------------- | ------------------------------ |
| Find the exit and get out of the maze | 40                             |
| Time                                  | <font color=#00000FF>10</font> |
| Number                                | <font color=#00000FF>10</font> |
| Auto identification and grab Pokemon  | 10                             |
| Auto driving and exploration          | 30                             |
| Multiple robots (bonus)               | <font color=#00000FF>10</font> |

1. Number: one valid catching gets 1 point until <font color=#00000FF>10 points</font>. Less than six gets 0 point.

2. Time: total <font color=#00000FF>10 points</font>. Every group has at most 7 minutes.

| Time (min)       | Points |
| ---------------- | ------ |
| <3               | 10     |
| 3$\leq$&&<5      | 8      |
| 5$\leq$&&$\leq$7 | 5      |

3. Strategy : we encourage you to use novel and high level techniques.

​	**Auto identification** means automatic identifying a Pokemon picture then modifying the pose and save a valid picture. 

​	**Auto driving** means you can not give any control to the robot during the game. 

​	**Multiple robots** means use more than one robots. 

​	<font color=#8A2BE2>**Note that whether you choose to use a single robot or multiple robots, you need to put them all at the entrance (the maze has two doors, you can cfontoose either one as the entrance).**</font>

4. <font color=#FF0000>Penalty</font>: any crash of the wall or two robots (if you use multi-robot) gets a **deduction of 5 points**.

## 3. Guidance of project

1) Firstly, you can refer to lab7. Write a node to recv the raw ros image and convert it into an opencv image using cvbridge, and then process it using the image processing algorithm in the opencv lib. 

2) If there is an identified target in the image, send a message to notify turtble3 to switch to grab mode . Once the target is grabbed , turtlebot3 continues to explore the maze to find an exit. 

3) There are many algorithms for maze exploration, including traditional algorithms and reinforcement learning algorithms.

4. Maybe You need to run a gmapping on you robot so that you can also know where you are. 

5) Once you meet a pokemon and want to catch it, the first thing you should do is to rotate your robot and let the pokemon picture fall in the middle of the frame. After that, the only thing you are expected to do is run a or a punch of node(s), which modifies the distance between the picture and robot until it meets a specific range and saves the image (with the red rectangle). 

6) Others... Provide pokemon_ws to everyone,please refer to the readme.pdf under the folder for details.