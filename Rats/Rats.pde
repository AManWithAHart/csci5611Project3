
Node[][] grid = new Node[40][40];
rat[] colony = new rat[1000];

player player1 = new player(50, 50, 10);
boolean [][] gridChecker = new boolean[40][40];

Vec2 acc = new Vec2(1,1);

int[] coord = new int[2];

void nodeSetter(){
  float a = 10;
  float b = 0;
  float r = 15;
  
  int numRats = 0;
  
  for(int i = 0; i < grid.length; i++){
    for(int j = 0; j < grid[0].length; j++){
          grid[i][j] = new Node(a,b,r,i,j, 0, 0);
          a += 20;     
    }
    a = 10;
    b += 20;
  }
}




void playerToNode(){
  int x = 0;
  int y = 0;
  float closest = 1000000000;
  for(int i = 0; i < grid.length; i++){
    for(int j = 0; j < grid[0].length; j++){
       float distance = player1.pos.distanceTo(grid[i][j].pos);
       if(distance < closest){
         closest = distance;
         x = i;
         y = j;
       } 
      
    }
  }
  coord[0] = x;
  coord[1] = y;
}



void distanceToPlayer(int i, int j, int dist){
  //if we oure out of bounds for index or if we have already checked this spot, dont do anything
  if((i < 0 || i >= grid.length) || (j < 0 || j >= grid[0].length)){}
  else if(gridChecker[i][j]){}
  else{
      //mark true, update the distance of the grid node
      gridChecker[i][j] = true;
      grid[i][j].dist = dist;
      
      //increase distance and then recursively call on 4 adjacent cells
      dist++;
      
      distanceToPlayer(i - 1, j, dist);
      distanceToPlayer(i + 1, j, dist);
      distanceToPlayer(i, j - 1, dist);
      distanceToPlayer(i, j + 1, dist);
  }
}


void closestNodeToRat(){
  for(int i = 0; i < grid.length; i++){
    for(int j = 0; j < grid[0].length; j++){
      for(int r = 0; r < colony.length; r++){
        //nullpointerexception
          if(colony[r].pos.distanceTo(grid[i][j].pos) < colony[r].pos.distanceTo(colony[r].closest.pos)){
               colony[r].closest = grid[i][j];
               grid[i][j].numRats++;
          }
      }
    }
  }
}

//RatPathing: Telling the rat which node to move towards.
void RatPathing(){
   for(int r = 0; r < colony.length; r++){
       if(colony[r].closest.dist == 0){}
       else{
          //Node dir = colony[r].closest;
          int i = colony[r].closest.i;
          int j = colony[r].closest.j;
          
          //check which adjacent node is closer to the player
          
          if(i - 1 >= 0){
             if(grid[i - 1][j].dist <  colony[r].closest.dist){
               colony[r].closest = grid[i-1][j];
             }
          }
          
          if(i + 1 <= grid.length){
            if(grid[i + 1][j].dist < colony[r].closest.dist){
               colony[r].closest = grid[i+1][j];
             }
          }
          
          if(j - 1 >= 0){
             if(grid[i][j - 1].dist < colony[r].closest.dist){
               colony[r].closest = grid[i][j - 1];
             }
          }
          
          if(j + 1 <= grid[0].length){
            if(grid[i][j + 1].dist < colony[r].closest.dist){
               colony[r].closest = grid[i][j + 1];
             }
          } 
          
       }
   }
}

// v * numRats/20 * dt


void scurry(){
   for(int r = 0; r < colony.length; r++){
      Vec2 distanceToCover = colony[r].closest.pos.minus(colony[r].pos);
      int numRats = colony[r].closest.numRats;
      
      colony[r].vel.add(distanceToCover);
      colony[r].vel.mul(numRats/20 * dt);
      
  
     
      
      colony[r].pos.add(colony[r].vel);
   }
}

void colonyGen(){
    float x = random(250.0, 750.0);
    float y = random(0.0, 250.0);
    
   for(int i = 0; i < colony.length; i++){
     colony[i] = new rat(x,y);
   }
    
}

void playerSet(){
  float x = random(250.0, 750.0);
  float y = random(500.0, 750.0);
  
  player1.pos.x = x;
  player1.pos.y = y;
}

void setup(){
  size(800, 800);
  colonyGen();
  playerSet();
  nodeSetter();
  //drawNodes();
}

float dt = 0.05;
void draw(){
  playerToNode();
  distanceToPlayer(coord[0], coord[1], 0);
  closestNodeToRat();
  scurry();
  
  fill(135,135,135);
  for(int i = 0; i < colony.length; i++){
    circle(colony[i].pos.x, colony[i].pos.y, 10);
  }
  
  fill(3,84,171);
  circle(player1.pos.x, player1.pos.y, 15);
  
  
}



//DEBUG: Showing where the nodes are on the map
void drawNodes(){
  fill(255,0,0);
  for(int i = 0; i < grid.length; i++){
    for(int j = 0; j < grid[0].length; j++){
        circle(grid[i][j].pos.x, grid[i][j].pos.y, grid[i][j].r);
    }
  }
}



public class player{
  float x, y, r;
  Vec2 pos;
  Vec2 vel;
  public player(float x, float y, float r){
    this.pos = new Vec2(x,y);
    this.r = r;
    this.vel = new Vec2(0,0);
  }
}


public class rat{
  Vec2 pos;
  Vec2 vel;
  Node closest;
  
  public rat(float x, float y){
    this.pos = new Vec2(x, y);
    this.vel = new Vec2(0,0);
  }
}


public class Node{
  float x, y, r; 
  int i, j, numRats;
  Vec2 pos; 
  int dist;
  
  public Node(float x, float y, float r, int i, int j, int numRats, int dist){
    this.x = x;
    this.y = y;
    this.pos = new Vec2(x, y);
    this.r = r;
    this.i = i;
    this.j = j;
    this.numRats = numRats;
    this.dist = dist;
  }
}

//Vector Library
//CSCI 5611 Vector 2 Library [Example]
// Stephen J. Guy <sjguy@umn.edu>

public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

float cross(Vec2 a, Vec2 b){
  return a.x*b.y - a.y*b.x;
}


Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}

float clamp(float f, float min, float max){
  if (f < min) return min;
  if (f > max) return max;
  return f;
}
