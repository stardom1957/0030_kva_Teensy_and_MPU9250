#ifndef _history_h
#define _history_h
/*
0. Creation 2024-03-13

>>>>>>>>>>>>>>>>>>>>>>>> into branch main
1. Tab history.h DONE
2. ajout des defines pour debug print NO
3. add quaternionFilters.ino : INTERESTING this file contains only code that is referenced in main and no include is necessary! DONE
4. add entire contents of /home/dominique/Arduino/robotics/IMUs/MPU9250_MS5637_AHRS_t3.ino into Teensy_and_MPU9250.ino DONE
5. modified wire statements to get clean compile ADD DONE
6. Amazon : ordered Adafruit PCD8544 NOKIA 5110 display (CANADUINO® Écran LCD 84 x 48 pixels - SPI - Rétroéclairage) X 2 RECEIVED 2024-03-19 DONE
7. Using contents of this directory (kva_Teensy_and_MPU9250); create new git repo and corresponding remote on github: FINISHED
 7.1 git init DONE
 7.2 git add . DONE
 7.3 git commit -m "Initial commit" DONE
 7.3 gh repo create and follow instructions using default options DONE
 7.4 git remote name is "kvat" DONE

git commit -m "Documenting git repo and remote creation"
>>>>>>>>>>>>>>>>>>>>>>>> branch main ADD COMMIT PUSHED
8. Connexion I2C for MPU9250: DONE
 see comments for pinout on Teensy

9. Connexion SPI for PCD8544 NOKIA 5110 display: DONE
 see comments for pinout on Teensy

10. Compile directive to eliminate pressure (MS5637) DONE
>>>>>>>>>>>>>>>>>>>>>>>> branch main TEST ADD
>>>>>>>>>>>>>>>>>>>>>>>> branch main commit -m "MPU 9250 9dof and Nokia 5110 display operational"
git remote "kvaimu" renamed "origin"
>>>>>>>>>>>>>>>>>>>>>>>> branch main pushed

11. add serial print control compile directives TEST ADD COMMIT -m "Issue 11: add serial print compile directives" PUSH DONE

>>>>>>>>>>>>>>>>>>>>>>>> begin branch button_control

12. insert code for button on interrupt: FINISHED
  12.1 one clic to continue on pause DONE
>>>>>>>>>>>>>>>>>>>>>>>> end branch button_control TEST ADD commit -m "Issue 12: button interrupts to get out of pause display"
branch button_control deleted

>>>>>>>>>>>>>>>>>>>>>>>> begin branch code_reorganize TEST FINISHED
13. reorganise code:
  13.1 create mpu9250_definitions.h for all definitions TEST ADD commit -m "mpu9250_definitions.h created and tested"
  13.2 create mpu9250_code.h and add include directive TEST DONE
  13.3 create i2c_general_functions.h and add include directive TEST DONE
  13.4 create MS5637_code.h and add include directive (set NOT to compile) TEST DONE
  13.5 ADD . DONE
>>>>>>>>>>>>>>>>>>>>>>>> end branch code_reorganize TEST ADD commit -m "code reorganize finished"


xx. Implement serial plot of YPR (see Kris Winer article)
xx. Set declination for home --> yaw   += 13.8f; // Declination 
*/

#endif