# Auto_drive_car
## 프로젝트 목적
초음파 센서와 DC motor를 이용하여 장애물을 피해 자율주행하는 자동차를 만듭니다.<br/> 
## 사용 디바이스
stm32f411, 초음파 센서, DC moter, 블루투스 모듈, 안드로이드 휴대폰
## 개발환경
STM32CubeIDE, arduino bluetooth application(휴대폰에 설치)
## High Level Design
<img src="./img_video/auto_drive_car.png">

## 수행역할
초음파 거리에 따른 모터 제어 알고리즘을 구현했습니다. 또한 초음파 센서의 거리 데이터를 가공하는 역할을 맡았습니다.  필터 알고리즘을 사용하여 거리 데이터에 발생한 오차를 줄였습니다.

## 시연영상
<img src="./img_video/auto_drive_car.gif" width=400 height=600>

[원본영상](https://github.com/BrotherHwan/Auto_drive_car/blob/main/img_video/auto_drive.mp4)(이 링크의 raw file 다운로드시 좀 더 크고 명확한 영상을 확인하실 수 있습니다. ) 
