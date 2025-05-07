# Final Project in Software Engineering Practical Course

> **Special thanks** to **Aleksandre Gordeladze** for his great effort in laying the foundation for this repository.

This repository provides a robust template for a ROS-based Duckiebot project with a focus on **lane following**. It serves as a foundation for more advanced autonomous behaviors such as **obstacle avoidance**, **object and sign detection**, and **navigation**.

Developed as part of the **Software Engineering Practical Course** at **Kutaisi International University**, this repository is designed to run inside the [Duckietown](https://www.duckietown.org/) ecosystem using Docker and `duckietown-shell` (`dts`).

---

## 🛠️ Project Structure

This template includes the essential components to get a Duckiebot to follow lanes autonomously:

- ✅ A **camera node** that processes real-time image input using OpenCV
- ✅ A **wheel control node** that computes and publishes motor commands
- ✅ A `default` launcher for easy deployment
- 🧱 Clean modular setup to extend with additional nodes or features

---

## 🚀 How to Use It

### 1. Clone the Repository

```bash
git clone https://github.com/nikakhalatiani/base-lane-follower.git
cd base-lane-follower
```

### 2. Make ROS Nodes Executable

Navigate to the `src/` folder and run:

```bash
chmod +x ./packages/Main/src/camera_node.py
chmod +x ./packages/Main/src/wheel_control_node.py
```

### 3. Build the Project

Every time you modify the code, rebuild the image using:

```bash
dts devel build -f
```

### 4. Run on Duckiebot

To deploy and run your code on a physical Duckiebot:

```bash
dts devel run -R BOTNAME -L default -X
```

> **Note:** `default` refers to the launch script in the `launchers/` directory and should not be renamed.

---

## 🧪 What to Expect

When deployed on the Duckiebot:
- The **camera node** captures and processes video feed in real time.
- It identifies **lane markings** (white and yellow).
- Motor commands are computed based on detected features and passed to the **wheel control node**.
- A **visual overlay** of processed output, including detected lines, is shown for debugging or demonstration.

---

## 📁 Folder Structure (Simplified)

```
.
├── launchers/
│   └── default.sh
├── packages/
│   └── Main/
│       ├── src/
│       │   ├── camera_node.py
│       │   └── wheel_control_node.py
│       └── ...
├── Dockerfile
├── README.md
└── ...
```

> All user-defined logic and ROS nodes should be placed inside `packages/Main/src`.

---

## 👨‍💻 Contributors

This repository is maintained as part of the **Software Engineering Practical Course** at **Kutaisi International University**.

> 💡 **Contributions are welcome!**  
> You are encouraged to:
> - **Fork** this repository and build on top of it
> - **Open issues** for bugs, questions, or feature requests
> - **Submit pull requests** (PRs) to enhance or fix the current implementation
