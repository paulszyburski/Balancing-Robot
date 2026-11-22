# Balancing-Robot
This project will consist of an AI model thats trained to balance on two wheels and then we will make a robot that actually balances using AI, in the future we could maybe make some deterministic bot and compare their accuraciues


For this project you will need:
- raspberry pi
- MPU-6050
- two motors with wheels(one with a encoder)
- level converter(for a safe connection between pi and encoder)
- 3d printer(or order a 3d printed file)


Here is how its goings:
At first i wrote the pybullet simualtion and then i implemented an AI that learned through reinforced learning. it worked well.
then i downloaded a open source stl file and 3d printed it. problem is, i wasnt really experienced and just didnt notice that my pi and mcore would fit on it. On my second try i 3d printed a big platform with 4 screw holes for the wheels. The problem was the small, thin plastic pieces for the wheels just couldnt hold the load. I also noticed that the AI had a really hard time balancing on that model compared to the first one. Currently im on my third try and i will 3d print a rlly solid piece of plastic and instead of 1 floor platform, i will make 2 platforms and i will place the batteries under the platform for a better center of mass