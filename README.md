\documentclass{article}
\usepackage[margin=1in]{geometry}
\usepackage{hyperref}
\usepackage{xcolor}
\usepackage{listings}
\usepackage{titlesec}

% Style for bash code blocks
\lstdefinestyle{bash}{
  backgroundcolor=\color{gray!10},
  basicstyle=\ttfamily\small,
  breaklines=true,
  frame=single,
  language=bash
}

\titleformat{\section}{\large\bfseries}{\thesection}{1em}{}

\begin{document}

\begin{center}
    {\huge \textbf{Multi-Robot Simulation with SLAM \& NAV2 (ROS 2 Humble)}}\\[1em]
    \textit{TurtleBot4 | Gazebo | Cartographer | Nav2 | Map Merging | RViz2}
\end{center}

\hrule
\vspace{1em}

\section*{📦 Installation Instructions}

\subsection*{1. Install ROS 2 Humble}
Follow the official \href{https://docs.ros.org/en/humble/Installation.html}{ROS 2 Humble installation guide}.  
Install the \textbf{desktop version} and source the setup script:
\begin{lstlisting}[style=bash]
source /opt/ros/humble/setup.bash
\end{lstlisting}

\subsection*{2. Install Gazebo Classic}
Install Gazebo Classic for simulation support:
\begin{lstlisting}[style=bash]
sudo apt update
sudo apt install gazebo
\end{lstlisting}

\subsection*{3. Install RViz2}
If not already installed:
\begin{lstlisting}[style=bash]
sudo apt install ros-humble-rviz2
\end{lstlisting}

\subsection*{4. Install TurtleBot4 Simulator}
\begin{lstlisting}[style=bash]
sudo apt install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes
\end{lstlisting}

\subsection*{5. Install Cartographer and Nav2}
\begin{lstlisting}[style=bash]
sudo apt install ros-humble-cartographer ros-humble-navigation2
\end{lstlisting}

\subsection*{6. Install Dependencies Using rosdep}
\begin{lstlisting}[style=bash]
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
\end{lstlisting}

\section*{🚀 Running the Simulation}

\subsection*{1. Launch Multi-Robot Simulation with SLAM \& Nav2}

\textbf{Launch Robot 1:}
\begin{lstlisting}[style=bash]
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py model:=lite namespace:=robot1 slam:=true nav2:=true x:=1.0 y:=0.0 
\end{lstlisting}

\textbf{Launch Robot 2:}
\begin{lstlisting}[style=bash]
ros2 launch turtlebot4_ignition_bringup turtlebot4_spawn.launch.py namespace:=robot2 x:=1.5 y:=0.0 model:=lite slam:=true nav2:=true
\end{lstlisting}

\subsection*{2. Visualize in RViz2}

\textbf{View Robot 1:}
\begin{lstlisting}[style=bash]
ros2 launch turtlebot4_viz view_model.launch.py namespace:=/robot1
\end{lstlisting}

\textbf{View Robot 2:}
\begin{lstlisting}[style=bash]
ros2 launch turtlebot4_viz view_model.launch.py namespace:=/robot2
\end{lstlisting}

\subsection*{3. Merge Maps}
\begin{lstlisting}[style=bash]
ros2 launch merge_map merge_map_launch.py
\end{lstlisting}

\subsection*{4. Save the Generated Map}
\begin{lstlisting}[style=bash]
ros2 run nav2_map_server map_saver_cli -f <map_name>
\end{lstlisting}

Replace \texttt{<map\_name>} with your preferred map file name.

\section*{📖 Notes}
\begin{itemize}
    \item Ensure all packages are properly sourced after installation.
    \item Adjust robot positions (\texttt{x} and \texttt{y} parameters) based on your environment.
    \item This setup uses \textbf{Gazebo Classic} --- for Ignition or Garden, adjust launch files accordingly.
\end{itemize}

\section*{📌 License}
This project is licensed under the \textbf{MIT License}.

\section*{🤖 Credits}
Built with ❤️ using ROS 2, Gazebo, Cartographer, Nav2, and TurtleBot4.

\end{document}
