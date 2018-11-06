#!/bin/bash
#This script for hot swapping git users
#The script depends on several txt files being in the same folder
#Nikolas Gamarra 11/3/18

#To run the script type "sudo ./setGitUser.sh"
#If you have trouble with this make sure the file properties/permissions allow it to be excutable. 

#location of ipconfig file
file='/etc/network/interfaces'

clear
tput setaf 1
cat ~/AVRS_ws/.git/config
echo Above is the current git user config
tput setaf 2
read -p "Enter a number 0-3 (0 -> Niko -+- 1 -> Ryan -+- 2 -> Remz -+- 3 -> Matt): " input

tput setaf 7 
case $input in
	0) 
	echo \ 
	echo setting user...
	sudo cp -f niko1499.txt ~/AVRS_ws/.git/config
	;;
	1) 
	echo \ 
	echo setting user...
	sudo cp -f Ryano647.txt ~/AVRS_ws/.git/config
	;;
	2) 
	echo \ 
	echo setting user...
	sudo cp -f jremz8902.txt ~/AVRS_ws/.git/config
	;;
	3) 
	echo \ 
	echo setting user...
	sudo cp -f mattfortmeyer.txt ~/AVRS_ws/.git/config
	;;
	
	*)echo error
	exit
	;;
esac

tput setaf 1
echo \ 

echo The git user config is now: 
tput sgr0
cat ~/AVRS_ws/.git/config
tput setaf 2



