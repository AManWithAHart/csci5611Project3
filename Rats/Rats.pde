


Node[][] grid = new Node[40][40];


void nodeSetter(Node[][] A){
  float a = 10;
  float b = 0;
  float r = 10;
  
  for(int i = 0; i < A.length; i++){
    for(int j = 0; j < A[0].length; j++){
          A[i][j] = new Node(a,b,r, 0);
          a += 20;     
    }
    a = 10;
    b += 20;
  }
}

void drawNodes(){
  fill(255,0,0);
  for(int i = 0; i < grid.length; i++){
    for(int j = 0; j < grid[0].length; j++){
        circle(grid[i][j].pos.x, grid[i][j].pos.y, grid[i][j].r);
    }
  }
}

void setup(){
  size(800, 800);
  
  nodeSetter(grid);
  //drawNodes();
}




public class player{
  float x, y, r;
  Vec2 pos;
  public player(float x, float y, float r){
    this.pos = new Vec2(x,y);
    this.r = r;
  }
}


public class rat{
  Vec2 pos;
  Node closest;
  
  public rat(float x, float y){
    this.pos = new Vec2(x, y);
  }
}


public class Node{
  float x, y, r;
  Vec2 pos; 
  int dist;
  
  public Node(float x, float y, float r, int dist){
    this.x = x;
    this.y = y;
    this.pos = new Vec2(x, y);
    this.r = r;
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
