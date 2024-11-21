# Auto_drive_car
## 프로젝트 소개
초음파 센서와 DC모터를 이용하여 주어진 코스를 자율주행하는 자동차를 만듭니다.<br/> 
## 구성인원
2명
## 기술 스택
![a](https://img.shields.io/badge/C-00599C?style=for-the-badge&logo=cpp&logoColor=white) ![b](https://img.shields.io/badge/STM32-03234B?style=for-the-badge&logo=stmicroelectronics&logoColor=white) ![c](https://img.shields.io/badge/STM32CubeIDE-03234B?style=for-the-badge&logo=stmicroelectronics&logoColor=white) ![d](https://img.shields.io/badge/ARM-FF6F00?style=for-the-badge&logo=arm&logoColor=white) ![e](https://img.shields.io/badge/RTOS-FF6F00?style=for-the-badge&logo=rtos&logoColor=white)
## 역할
초음파 거리에 따른 모터 제어 알고리즘을 구현했습니다. 또한 초음파 센서의 거리 데이터를 가공하는 역할을 맡았습니다. 이동평균필터 알고리즘을 사용하여 거리 데이터에 발생한 오차를 줄였습니다.
## 문제
1. 초음파 센서의 거리데이터 오차발생
2. 반응 시간의 지연으로 인한 장애물과의 충돌
## 문제 해결 방안
1. 1) 모터와 센서가 가까운 상태에서 모터구동을 하면 자기장의 영향으로 센서 데이터가 크게 요동치는 것을 확인하였습니다. 그래서 모터와 센서의 거리를 어느정도 두도록 설계했습니다.
   2) 그럼에도 한 번씩 발생하는 오차값은 이동평균필터를 사용하여 보정했습니다.
2. RTOS를 활용했습니다. 실시간성을 확보할 수 있었고 그로 인해 장애물과의 충돌횟수를 줄일 수 있었습니다.
## 결과
- 초음파센서의 거리 데이터 정확도 60% 개선
- 평균 충돌 횟수 86.6% 감소
## 고찰
- 코스에 한번도 부딛히지 않고 통과할 확률은 60%였습니다. 40%정도는 1~2번 정도 부딛히고 통과했습니다.
- 초음파 신호의 상승엣지와 하강엣지에 대한 타이머를 분리하여 반응성을 좀 더 개선했었으면 하는 아쉬움이 있습니다.
- DMA방식을 사용하여 CPU의 부하를 줄이면서 반응성을 개선했었으면 하는 아쉬움이 있습니다.
## High Level Design
<img src="./img_video/auto_drive_car.png">

## 시연영상
<img src="./img_video/auto_drive_car.gif" width=400 height=600>

[원본영상](https://github.com/BrotherHwan/Auto_drive_car/blob/main/img_video/auto_drive.mp4)(이 링크의 raw file 다운로드시 좀 더 크고 명확한 영상을 확인하실 수 있습니다. ) 
