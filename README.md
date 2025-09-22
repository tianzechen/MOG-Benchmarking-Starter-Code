# MOG Benchmarking – Starter Code  

[![Python](https://img.shields.io/badge/python-3.7%2B-blue.svg)](https://www.python.org/)  
[![CoppeliaSim](https://img.shields.io/badge/CoppeliaSim-4.4.0-orange.svg)](https://www.coppeliarobotics.com/)  
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)  
[![Contributions](https://img.shields.io/badge/contributions-welcome-brightgreen.svg)](#-contributing)  

This repository provides **starter code** for benchmarking **multi-object grasping (MOG)** tasks in simulation.  
It demonstrates grasping and transferring objects from a source bin to conveyor belt bins using a **stochastic finger movement strategy** given any pre-grasp pose.  

Researchers and developers are encouraged to extend this starter code with their own control algorithms, grasping strategies, and benchmarking methods.  ![example](https://github.com/user-attachments/assets/a98c773f-c451-45bb-85c8-9a8df347db80)


---


## ✨ Features
- Integration with **CoppeliaSim 4.4.0** for physics-based robot simulation.  
- Starter implementation of a **stochastic movement strategy** for robot fingers.  
- Example simulation scene (`40mm_sphere.ttt`) included.  
- Modular Python API for extending and customizing experiments.  

---

## 📦 Requirements
- [CoppeliaSim](https://www.coppeliarobotics.com/downloads) **version 4.4.0**  
- Python ≥ 3.7

---

## 🚀 Getting Started

1. Clone the repository:
   ```bash
   git clone https://github.com/tianzechen/MOG-Benchmarking-Starter-Code.git
   cd MOG-Benchmarking-Starter-Code
2. Launch CoppeliaSim with the provided scene
3. Run the Starter code:
   ```bash
   python starter_code.py
4. Observe the robot grasping and transferring objects to the conveyor bins

## 📊 Benchmarking
This repo is intended as a foundation for **MOG benchmarking**. Future extensions may include:  
- Performance metrics (success rate, grasp efficiency, transfer throughput).  
- Comparisons between different grasp strategies.  
- Visualization and logging utilities.

---

## 📂 Repository Structure

```
MOG-Benchmarking-Starter-Code/
├── starter_code.py       # Main script with stochastic finger movement strategy
├── 40mm_sphere.ttt       # Example CoppeliaSim scene file (40mm sphere object)
├── PythonAPI/            # Python API bindings for CoppeliaSim
│   └── ...               # (place your API scripts here)
├── README.md             # Project documentation
└── LICENSE               # MIT license file
```

---

### 📜 License
```markdown
## 📜 License
This project is licensed under the [MIT License](LICENSE).
