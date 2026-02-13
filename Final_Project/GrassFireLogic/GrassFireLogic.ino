#include <Arduino.h>
#include <queue>

#define ROWS 5
#define COLS 9

// Map representation: 0 = empty, 99 = obstacle, 100 = Start, 101 = Goal
int8_t grid[ROWS][COLS] = {
  {0, 0, 0, 0, 0, 0, 0, 0, 101},
  {0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 99, 99, 0, 0, 0},
  {0, 0, 0, 0, 99, 99, 0, 0, 0},
  {100, 0, 0, 0, 0, 0, 0, 0, 0}
};

int distGrid[ROWS][COLS];

struct Point {
  int r, c;
};

void runGrassfire() {
  std::queue<Point> q;
  Point start, goal;

  // Initialize distance grid
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      distGrid[r][c] = -1;
      if (grid[r][c] == 101) {
        goal = {r, c};
        distGrid[r][c] = 0;
      } else if (grid[r][c] == 100) {
        start = {r, c};
      }
    }
  }

  q.push(goal);

  // Spread the fire
  int dr[] = {-1, 1, 0, 0};
  int dc[] = {0, 0, -1, 1};

  while (!q.empty()) {
    Point curr = q.front();
    q.pop();

    for (int i = 0; i < 8; i++) { // Changed from 4 to 8
      int nr = curr.r + dr[i];
      int nc = curr.c + dc[i];

      if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS) {
        // Check if it's not an obstacle and hasn't been visited
        if (distGrid[nr][nc] == -1 && grid[nr][nc] != 99) {
          
          // DIAGONAL CORNER CUTTING GUARD (Optional)
          // Prevents the "fire" from squeezing between two diagonal obstacles
          if (i >= 4) { 
              if (grid[curr.r][nc] == 99 && grid[nr][curr.c] == 99) continue;
          }

          distGrid[nr][nc] = distGrid[curr.r][curr.c] + 1;
          q.push({nr, nc});
        }
      }
    }
  }

  printResults(start, goal);
}

void printResults(Point start, Point goal) {
  Serial.println("Distance Map (The 'Fire'):");
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      if (distGrid[r][c] == -1) Serial.print("XX\t");
      else {
        Serial.print(distGrid[r][c]);
        Serial.print("\t");
      }
    }
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor
  runGrassfire();
}

void loop() {}