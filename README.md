# Auto_drive_car
## 프로젝트 소개
초음파 센서와 DC모터를 이용하여 주어진 코스를 자율주행하는 자동차 제작<br/> 
## 구성인원
2명
## 기술 스택
![a](https://img.shields.io/badge/C-00599C?style=for-the-badge&logo=cpp&logoColor=white) ![b](https://img.shields.io/badge/STM32-03234B?style=for-the-badge&logo=stmicroelectronics&logoColor=white) ![c](https://img.shields.io/badge/STM32CubeIDE-03234B?style=for-the-badge&logo=stmicroelectronics&logoColor=white) ![d](https://img.shields.io/badge/ARM-FF6F00?style=for-the-badge&logo=arm&logoColor=white) ![e](https://img.shields.io/badge/RTOS-FF6F00?style=for-the-badge&logo=rtos&logoColor=white)
## 역할
-초음파 거리에 따른 모터 제어 알고리즘을 구현 
-초음파 센서의 거리 데이터 신뢰성 향상을 위한 시스템 구성
-거리 데이터 가공
## 문제
1. 초음파 센서의 거리데이터 오차발생
2. 반응 시간의 지연으로 인한 장애물과의 충돌
## 문제 해결 방안
1. 1) 모터와 센서가 가까운 상태에서 모터구동을 하면 자기장의 영향으로 센서 데이터가 크게 요동치는 것을 확인 후 모터와 센서의 거리를 어느정도 두도록 설계
   2) 그럼에도 한 번씩 발생하는 오차값은 이동평균필터를 사용하여 보정
2.  RTOS를 활용으로 실시간성을 확보 및 장애물과의 충돌횟수 감소, 초음파 센서의 상승엣지와 하강엣지 타이머를 분리하여 반응성 개선
## 결과
- 초음파센서의 거리 데이터 정확도 60% 개선
- 평균 충돌 횟수 86.6% 감소
## 고찰
- DMA방식을 사용하여 CPU의 부하를 줄이면서 반응성을 개선했었으면 하는 아쉬움
## High Level Design
<img src="./img_video/auto_drive_car.png">

## 시연영상
<img src="./img_video/auto_drive_car.gif" width=400 height=600>

[원본영상](https://github.com/BrotherHwan/Auto_drive_car/blob/main/img_video/auto_drive.mp4)(이 링크의 raw file 다운로드시 좀 더 크고 명확한 영상을 확인하실 수 있습니다. ) 
