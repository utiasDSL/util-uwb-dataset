#!/bin/bash
RED='\033[0;31m' BLUE='\033[0;34m' GREEN='\033[0;32m' NC='\033[0m' # No Color
read -s -p "Please enter password for sudo: " sudoPW
echo $sudoPW | sudo add-apt-repository universe
echo -en "\e[1A"
echo -e "\e[0K"
echo $sudoPW | sudo apt update
echo -en "\e[1A"
echo -e "\e[0K"
echo $sudoPW | sudo apt install -y p7zip-full
echo -en "\e[1A"
echo -e "\e[0K"
echo $sudoPW | sudo apt install curl
echo -en "\e[1A"
echo -e "\e[0K"
echo | pip3 install pyquaternion
echo -en "\e[1A"
echo -e "\e[0K"
echo | pip3 install numpy scipy matplotlib==3.4.2 pyyaml pandas
echo -en "\e[1A"
echo -e "\e[0K"
echo | pip3 install -U scikit-learn
echo -en "\e[1A"
echo -e "\e[0K"
echo -e "${BLUE}Grab a cup of coffee and relax${NC}\n"
echo | curl -L https://github.com/utiasDSL/util-uwb-dataset/releases/download/dataset-v1.0/dataset.7z > dataset.7z
# echo | mkdir dataset
# echo | 7z x ./dataset.7z -o./dataset/
echo | 7z x ./dataset.7z 
echo | rm dataset.7z
echo -e "${GREEN}Setup complete${NC}\n"



