# 🎮 Pac-Man Game in Python

A classic **Pac-Man-inspired arcade game built using Python**.
The player controls Pac-Man through a maze, collects pellets, avoids ghosts, and tries to achieve the highest possible score.

## 🕹️ Features

* 🎯 Classic Pac-Man-style gameplay
* 👻 Multiple ghosts
* 🟡 Pellet/coin collection
* 🧱 Maze-based movement
* ❤️ Lives system
* 📈 Score tracking
* ⌨️ Keyboard controls
* 🚀 Adjustable player and ghost speed
* 🔄 Game restart functionality
* 🎮 Smooth real-time gameplay

## 🛠️ Technologies Used

* **Python 3**
* **Pygame**
* Object-Oriented Programming (OOP)
* Collision Detection
* Game Loop
* Keyboard Event Handling

## 📂 Project Structure

```text
PACMAN_2/
│
├── main.py              # Main game file
├── assets/              # Images, sounds and other game assets
│   ├── images/
│   └── sounds/
│
├── README.md            # Project documentation
└── requirements.txt     # Required Python libraries
```

> The exact file structure may vary depending on the version of the project.

## ⚙️ Installation

### 1. Clone the repository

```bash
git clone https://github.com/waninayan22-tech/PACMAN_2.git
```

### 2. Navigate to the project folder

```bash
cd PACMAN_2
```

### 3. Install the required library

```bash
pip install pygame
```

Or, if a `requirements.txt` file is provided:

```bash
pip install -r requirements.txt
```

### 4. Run the game

```bash
python main.py
```

## 🎮 Controls

| Key                | Action     |
| ------------------ | ---------- |
| ⬆️ W / Up Arrow    | Move Up    |
| ⬇️ S / Down Arrow  | Move Down  |
| ⬅️ A / Left Arrow  | Move Left  |
| ➡️ D / Right Arrow | Move Right |
| `ESC`              | Quit Game  |

## 🧠 How the Game Works

The game continuously runs inside a **game loop**.

The basic process is:

```text
Start Game
    ↓
Initialize Player, Ghosts & Maze
    ↓
Read Keyboard Input
    ↓
Move Pac-Man
    ↓
Move Ghosts
    ↓
Check Collisions
    ↓
Update Score / Lives
    ↓
Render Game
    ↓
Repeat
```

### Pac-Man

The player controls Pac-Man using the keyboard. Pac-Man can move through the available paths in the maze and collects pellets to increase the score.

### Ghosts

Ghosts move around the maze and act as obstacles. If Pac-Man collides with a ghost, a life is lost.

### Scoring

The score increases when Pac-Man collects pellets.

Example:

```text
Pellet collected → Score increases
Ghost collision   → Life decreases
All pellets       → Level/Game completion
```

## 🚀 Future Improvements

Some possible improvements for future versions include:

* [ ] Add multiple levels
* [ ] Implement advanced ghost AI
* [ ] Add power pellets
* [ ] Add different ghost behaviors
* [ ] Add background music and sound effects
* [ ] Add high-score saving
* [ ] Add difficulty levels
* [ ] Add improved animations
* [ ] Add start/pause/game-over screens
* [ ] Add a leaderboard

## 📸 Screenshots

Add screenshots of your game here:

```markdown
![Pac-Man Gameplay](assets/images/screenshot.png)
```

## 🎯 Learning Objectives

This project helps demonstrate practical implementation of:

* Python programming
* Pygame development
* Object-oriented programming
* Game loops
* Event handling
* Collision detection
* 2D movement
* Basic game AI
* File and asset management

## 👨‍💻 Author

**Nayan Wani**

GitHub: [waninayan22-tech](https://github.com/waninayan22-tech)

## ⭐ Contributing

Contributions and suggestions are welcome.

If you would like to improve the project:

1. Fork the repository
2. Create a new branch
3. Make your changes
4. Commit your changes
5. Open a Pull Request

## 📜 License

This project is created for **educational and learning purposes**.

---

⭐ If you like this project, consider giving the repository a **star**!
