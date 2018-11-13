#AVRS-MQP

Advisors: Craig Putnam, Jie Fu
Students: Matthew Fortmeyer, Nikolas Gamarra, Ryan O’Brien, Jacob Remz


## Hey lets use markdown so it looks nice on Git

```
code
```

```
git config --global user.email "MY_NAME@example.com"
```

git log --pretty="%an %ae%n%cn %ce" | sort | uniq
git log | grep Author: | sort | uniq


git commit --author="niko1499 <nxgamarra@gmail.com>" -m "commit as niko test" -a


to visualize the robot run

roslaunch abb_irb1600_support robot_state_visualize_irb1600_6_12.launch J23_coupled:="false" robot_ip:=192.168.100.100 

roslaunch abb_irb1600_6_12_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=192.168.100.100




