# Jason's Notebook (Group 43)
## Date: 2/11/25
- https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/adc.html#minimizing-noise - for information regarding the ESP32S3's ADC
- https://www.digikey.com/en/products/detail/susumu/RG3216P-5004-B-T1/7035382 - 5 MOhm resistor
- https://www.digikey.com/en/products/detail/susumu/RGV3216P-3004-B-T1/13180848 - 3 MOhm resistor
- https://www.digikey.com/short/t3m7z5mp - 500kOhm resistor
- https://www.digikey.com/short/5ht31h15 - 3v3 TVS diode
- Block diagram: ![image](https://github.com/user-attachments/assets/25da4bd7-b4c6-40c7-ba9c-9ddc89bf4401)
<img width="574" alt="image" src="https://github.com/user-attachments/assets/7160957d-d0a0-4c18-93dd-dadad4ced060" />

[Falstad - Intial voltage divider design](https://www.falstad.com/circuit/circuitjs.html?ctz=CQAgjCAMB0l3BWcMBMcUHYMGZIA4UA2ATmIxAUgpABZsKBTAWjDACgAlcDQ8FPbrzxURtKigTQUUGTARsAToL4CwPZVUrw4i5Wt76QE3pu2Q2Ac2XHreASLYAHIzSqGUrowl71RYMzrONMLKweLeIL4y-gG6NGAG6vG8NqZmcQkqtJmGVLjpXMlZRSGinjRSslDQ8gDueuoeburm9U3K2GgabNg0IABeDAB2DApM2ND0EGDQGGB0CPxgxEvwKMzk4rABO3h9E1Ns-ZFdNp3NKQPDo+OTbPVFhmHdDyGG3hdQRxSE4bwf1iuIzGB3u1gi7VSYMhEXOXhMYLhNiKUIemWRIVRtExEQBUKAA)

## Date: 2/26/25
Previous week's notebook was lost due to forgeting to commit changes
Below are the power and input/output connectors for the board
![image](https://github.com/user-attachments/assets/3f07a5b6-8017-44ec-b584-96873469a5bc)
![image](https://github.com/user-attachments/assets/38165c13-722d-4e35-904f-fa8baf87cd60)

Additionally, purchasing first webcam: [https://a.co/d/4ZRPdg5](url)

Autonomy subsystem flow chart: ![image](https://github.com/user-attachments/assets/8ce12c57-3cf0-42d1-8ecd-d9f75b5947fb)

Using KiCAD to develop first PCB order:
Schematic:

![image](https://github.com/user-attachments/assets/500edefc-73e3-42d7-b81c-06eca37276cb)
PCB:

![image](https://github.com/user-attachments/assets/7a489d30-9504-443f-9136-bbccd5302bca)

Sending first order info to Michael to purchase digikey parts

Pure-pursuit information:
[https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit ](url)

Better flowchart for design doc: 
![image](https://github.com/user-attachments/assets/770975fd-4c74-4e3f-90d1-029f29e54410)

## Date: 3/7/25
Topic: Changing connection protocol
TRIED USING BLUETOOTH - BLUETOOTH LOW ENERGY IS SLOW AND REQUIRES CONSTANT SIGNALS
TRIED USING WIFI - WIFI PING HAS LARGE JITTER
SWITCHING TO ESPNOW
Latency tests: [https://github.com/QinghuaYao/ECE445-notebook/blob/main/Jason/latency.py](url)
DEBUG:
- ISSUE: Boards wouldn't connect
- SOLUTION: Exchange MAC addresses properly, set them hard in code, no need to change them
- ISSUE: latency signals sent getting lost, so measurement is incorrect (serial would simply read the signals sent from the server, not the client)
- SOLUTION: require custom ACK signals to be sent from each board, so serial can distinguish what is being sent and what is being received
Testing: ![image](https://github.com/user-attachments/assets/23f3723d-6292-4472-b593-a1283139c8f9)


First breadboard demo - getting apriltag conversion working: [https://github.com/QinghuaYao/ECE445-notebook/blob/main/Jason/april_tag.py](url)

DEBUG:
- ISSUE: ghost tags being recognized everywhere
- SOLUTION: use decision boundary to remove ghost tags
Camera Parameters obtained using:

[https://pub.towardsai.net/camera-calibration-using-opencv-984775433343](url)
[https://alphapixeldev.com/opencv-tutorial-part-1-camera-calibration/](url)
[https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html](url)

NexiGo Webcam params: fx, fy, cx, cy = 850, 850, 740, 425

## Date: 3/14/25
Topic: completed first tests of simulation
![image](https://github.com/user-attachments/assets/e6af5b22-9c21-4ae9-84dd-64f10ee3506a)
DEBUG (apriltags):
- ISSUE: timing is super slow (20ms per pass)
- SOLUTION: use quad_decimate (0.5 for higher speed)

Environment is in battlebots_env.py: [https://github.com/QinghuaYao/ECE445-notebook/blob/main/Jason/battlebots_env.py](url)
DEBUG (environment):
genuinely, countless issues
the main ones:
- ISSUE: physics didn't make sense
- ISSUE: user control was broken
- ISSUE: robot would not drive even when given commands
- ISSUE: robot would drive way too fast
- ISSUE: collision wouldn't work (robots would phase through each other)
most have been solved by trial and error

## Date: 3/18/25
Topic: completed pure-pursuit algorithm: [https://github.com/QinghuaYao/ECE445-notebook/blob/main/Jason/battlebots_purepursuit.py](url)
DEBUG (algo):
- ISSUE: left was right, right was left
- SOLUTION: core switch, had to standardize direction between environment 
- ISSUE: adjustments to the actual speed of the robot, since left and right drive were going too fast
- SOLUTION: tweaking velocity values in battlebots_environment

Output can be logged each state
Total time average for entire algorithm: 60.015714285714274 us
Adjusting flowchart for anti-weapon-on-weapon setup: 
![image](https://github.com/user-attachments/assets/0275434f-f120-4354-b499-2c1b3a12330e)
![image](https://github.com/user-attachments/assets/f009e6cf-7cad-472f-a846-1d25df76022c)

Also diagrams/formulas for IPR:
![image](https://github.com/user-attachments/assets/651e7012-c029-4e44-b2ae-8ab52caceef8)
![image](https://github.com/user-attachments/assets/40982b9f-5a9a-4fc9-a0be-8a06a8b985c8)
![image](https://github.com/user-attachments/assets/448896d2-e91f-46ac-a69f-41ee1b3f1e4a)
![image](https://github.com/user-attachments/assets/b3406b9d-55d1-4cf0-a025-bdbb35193f33)
![image](https://github.com/user-attachments/assets/319fa073-3758-47c1-a4e6-d7dbaba43757)

## Date: 3/25/25
Topic: issues with hardware, need to fix, got signals sent to board
Redid the KiCAD partially helping Qing, switching to use of the USB-to-serial converter
Finalized block diagram: ![image](https://github.com/user-attachments/assets/e16c0878-f307-44ee-bf42-1893786b7aa4)
Also switching to the LIS331 from MPU6050, was informed that it's pretty good: [https://www.st.com/resource/en/datasheet/lis331dlh.pdf](url)
We should be fine, with time to finish before mid-April
Seeed Studio Xiao ESP32S3 pinout ![image](https://github.com/user-attachments/assets/e006f1a8-3120-43a0-ba6e-544467f3d3bb)
Arduino client (on-robot code): [https://github.com/QinghuaYao/ECE445-notebook/blob/main/Jason/arduino_client.ino](url)
Arduino server (from computer): [https://github.com/QinghuaYao/ECE445-notebook/blob/main/Jason/arduino_server.ino](url)
DEBUG (client):
too many
- ISSUE: MCPWM unit 0 doesn't work
- SOLUTION: investigation leads to seeed studio xiao not actually supporting MCPWM unit 0, switching all other timers to 0
- ISSUE: TIMER doesn't properly time
- SOLUTION: investigation leads to ESP32 library using an outdated version of the ESP32S3 hardware timers, have to guess and check actual numbers
- ISSUE: reading input PWM doesn't work, signals stall out due to ISRs not firing if no signal is received
- SOLUTION: add watchdog to reset signals back to 0 if no input is received
- ISSUE: 30 different issues with the Wifi signals, and threading
- SOLUTION: lots of documentation reading [https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/index.html](url)
DEBUG (server):
too many as well
- ISSUE: not connected multiple times
- SOLUTION: switch to connecting using MAC address as peers directly instead of searching for peers
- ISSUE: sending signals ends up with signals arriving in incorrect order
- SOLUTION: either use ACK with custom signals for each input, or send all of them at once using a custom pwm_data struct

Added pure-pursuit information, by simming the next motions of the robot in advance (very little drain, 3ms of CPU use for 300 steps)
Video of it all working: [https://drive.google.com/file/d/1QAclTgO31wBWgzG7DGrPYsUrj7KnDfsV/view?usp=sharing](url)

## Date: 4/1/25
Topic: 3/4 subsystems working
Connection from virtual environment -> algorithm -> robot established, video link below: 
[https://drive.google.com/file/d/18kYJzT2jaFrgjoZz8jekUooTfdeGOrBi/view?usp=sharing](url)
Using ESP-NOW, latency is super low!!
We are able to drive ESCs using MCPWM hardware: [https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/api-reference/peripherals/mcpwm.html](url)

## Date: 4/8/25
Post-Robobrawl: CRACK is unbelievably damaged - near irreperable
focus moves from autonomy to making the robot drive again
![image](https://github.com/user-attachments/assets/b5ec94b8-56e0-4f3c-8e0a-b2ebdac5825f)
center weapon rail is mangled, right side weapon rail is completely torn
considering replacements with PLA printed parts

## Date: 4/15/25
Topic: PLA conversion failure
![image](https://media.discordapp.net/attachments/814909747706462260/1370123065257889853/image0.jpg?ex=681e5a0a&is=681d088a&hm=aac4b4ceea79bd5510ced781be731192643992a7359aa4971fff8b93a78785de&=&format=webp&width=1689&height=1267)
PLA is not strong enough to properly handle a weapon fire test, must consider an alternative solution to running the robot
Purchasing an RC car for mock demo: [https://a.co/d/cZ9rRtn](url)
Hardware has not been working either, they need to buy a stencil?

## Date: 4/22/25
Topic: Mock Demo work
Weapon will be removed as compromise, will be switching to drive only
Had to go to library multiple times, switching the tags from 25h9 to 16h5
Apriltag recognition performed decently well, will need to still use Optical Flow for anything after demo

## Date: 4/23/25
Topic: Completed Mock Demo
Demo link: [https://drive.google.com/file/d/1CyM5mAFQG4s7LUPjuEVyyVv1AHwV2Kl4/view?usp=sharing](url)
Deviation Verification: ![image](https://github.com/user-attachments/assets/e1b4d4ab-bcf5-4be7-8369-41a354848b30)
Autonomous step timing: ![image](https://github.com/user-attachments/assets/45b186ef-0260-46a7-acea-c410e1226625)

Hopefully hardware gets completed by the time I can resume work on Monday

## Date: 4/28/25
Topic: Night before demo
Issues with hardware still, using optical flow and SolvePnP, finalized battlebots environment in battle_env: [https://github.com/QinghuaYao/ECE445-notebook/blob/main/Jason/battle_env.py](url)
DEBUG (battle_env):
- ISSUE: camera doesn't connect
- SOLUTION: iterate through all cameras until you find the correct camera (my laptop has like 5 different camera applications)
- ISSUE: camera takes >30 secs to initialize
- SOLUTION: {NEVER SOLVED}
- ISSUE: solvePnP requires multiple points
- SOLUTION: measure all 9 points (top left to bottom right)
Porting was not successful, the serial connection was damaged, we'll see 



