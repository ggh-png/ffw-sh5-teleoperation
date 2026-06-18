# FFW-SH5 Teleoperation Simulation

ROBOTIS AI Worker FFW-SH5 로봇의 물리 기반 텔레오퍼레이션 시뮬레이터.

MuJoCo / Gazebo 없이 **C++ + OpenGL 4.3 + Bullet3** 로 직접 구현.  
수학 라이브러리(Vec3/Mat4/Quaternion)와 FK/IK 엔진을 외부 라이브러리 없이 직접 작성.

![status](https://img.shields.io/badge/phase-1%20rendering-blue)

---

## 데모

| 조작 | 동작 |
|------|------|
| 마우스 좌클릭 드래그 | 카메라 회전 |
| 마우스 스크롤 | 줌 인/아웃 |
| 마우스 미들 드래그 | 카메라 팬 |
| `WASD` | 베이스 전후좌우 이동 |
| `Q` / `E` | 리프트 상승 / 하강 |
| `TAB` | 관절 선택 전환 |
| `↑` / `↓` | 선택 관절 각도 조절 |
| `ESC` | 종료 |

---

## 시스템 요구사항

- Ubuntu 22.04 / 24.04 (Wayland 또는 X11)
- NVIDIA GPU (OpenGL 4.3 이상) 또는 Mesa 소프트웨어 렌더러
- GCC 12+ 또는 Clang 15+
- CMake 3.20+

---

## 설치 및 빌드

### 1. 저장소 클론

```bash
git clone --recurse-submodules https://github.com/ggh-png/ffw-sh5-teleoperation.git
cd ffw-sh5-teleoperation
```

> `--recurse-submodules` 를 빠뜨렸다면:
> ```bash
> git submodule update --init --recursive
> ```

### 2. 의존성 설치

```bash
sudo apt-get update
sudo apt-get install -y \
    cmake build-essential \
    libglfw3-dev \
    libepoxy-dev \
    libbullet-dev \
    libtinyxml2-dev \
    libgl1-mesa-dev
```

### 3. 빌드

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

빌드 결과물: `build/ffw_teleop`

---

## 실행

### X11 / 일반 데스크톱

```bash
cd build
./ffw_teleop
```

### Wayland (GNOME / KDE)

```bash
cd build
DISPLAY=:0 XAUTHORITY=$XAUTHORITY ./ffw_teleop
```

> XAUTHORITY 경로가 다를 경우 확인:
> ```bash
> echo $XAUTHORITY
> ```

### 다른 MJCF 파일 지정

```bash
./ffw_teleop path/to/robot.xml
```

기본값: `assets/ffw_sh5/robotis_ffw/ffw_sh5.xml`

---

## 프로젝트 구조

```
ffw-sh5-teleoperation/
├── CMakeLists.txt
├── assets/
│   ├── shaders/
│   │   ├── phong.vert          # Blinn-Phong 버텍스 셰이더
│   │   └── phong.frag          # Blinn-Phong 프래그먼트 셰이더
│   └── ffw_sh5/                # ROBOTIS menagerie (git submodule)
│       └── robotis_ffw/
│           ├── ffw_sh5.xml     # MJCF 로봇 정의
│           └── assets/         # STL 메시
├── src/
│   ├── main.cpp
│   ├── math/                   # 직접 구현 — 헤더 온리
│   │   ├── Vec3.hpp / Vec4.hpp
│   │   ├── Mat4.hpp            # Column-major, lookAt/perspective 포함
│   │   ├── Quaternion.hpp      # w x y z (MuJoCo 컨벤션)
│   │   └── Transform.hpp
│   ├── robot/                  # FK / IK 라이브러리 — 직접 구현
│   │   ├── Joint.hpp           # Revolute / Prismatic / Free
│   │   ├── SceneNode.hpp       # 씬 그래프 노드
│   │   ├── ForwardKinematics.hpp  # DFS 트리 순회
│   │   ├── InverseKinematics.hpp  # Jacobian Transpose IK
│   │   └── RobotModel.hpp
│   ├── io/
│   │   ├── STLLoader.hpp/.cpp  # Binary STL 직접 파싱
│   │   └── MJCFParser.hpp/.cpp # MJCF XML → SceneNode 트리
│   ├── render/
│   │   ├── ShaderProgram.hpp/.cpp
│   │   ├── Mesh.hpp/.cpp       # VAO / VBO / EBO
│   │   ├── Camera.hpp          # Orbit 카메라
│   │   └── Renderer.hpp/.cpp
│   ├── physics/
│   │   └── PhysicsWorld.hpp/.cpp  # Bullet3 래핑
│   ├── input/
│   │   └── InputManager.hpp/.cpp
│   └── ui/
│       └── JointPanel.hpp/.cpp # Dear ImGui 관절 패널
└── third_party/
    └── imgui/                  # Dear ImGui (git submodule)
```

---

## 외부 라이브러리

| 라이브러리 | 용도 | 설치 방법 |
|-----------|------|----------|
| GLFW 3 | 윈도우 + 입력 | apt |
| libepoxy | OpenGL 함수 로더 (Wayland 호환) | apt |
| Bullet3 | 물리 엔진 (중력, 충돌) | apt |
| tinyxml2 | MJCF XML 파싱 | apt |
| Dear ImGui | 디버그 UI | git submodule |

수학 라이브러리, FK/IK, STL 로더, MJCF 파서는 **직접 구현**.

---

## 개발 현황

- [x] Phase 1 — 프레임워크 셋업 & 렌더링 파이프라인
  - [x] GLFW + OpenGL 4.3 + libepoxy (Wayland 지원)
  - [x] MJCF 파서 (meshdir, scale, joint limits 포함)
  - [x] Binary STL 로더
  - [x] Phong 셰이더 + Orbit 카메라
  - [x] Bullet3 지면 + 베이스 Kinematic Body
  - [x] Dear ImGui 관절 슬라이더
- [ ] Phase 2 — FK 시각화 검증
- [ ] Phase 3 — 게임패드 텔레오퍼레이션 + Jacobian IK
- [ ] Phase 4 — 그림자, 성능 최적화

---

## 라이선스

MIT
