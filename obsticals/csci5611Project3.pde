boolean pointLineCollision(Vec2 p, Vec2 v1, Vec2 v2) {
    float len = v1.distanceTo(v2);
    float d1 = p.distanceTo(v1);
    float d2 = p.distanceTo(v2);
    if (d1 + d2 >= len - 0.01 && d1 + d2 <= len + 0.01) {
        return true;
    }
    return false;
}

boolean lineBallCollision(Vec2 v1, Vec2 v2) { //https://www.jeffreythompson.org/collision-detection/line-circle.php
    if (v1.distanceTo(obstical_Pos) < obstical_Radius || v2.distanceTo(obstical_Pos) < obstical_Radius) { //check if line exists within circle
        return true;
    }
    float len = v1.distanceTo(v2);
    float t = (((obstical_Pos.x - v1.x) * (v2.x - v1.x)) + ((obstical_Pos.y - v1.y) * (v2.y - v1.y))) / pow(len, 2);
    Vec2 nearestPoint = new Vec2(v1.x + (t * (v2.x - v1.x)), v1.y + (t * (v2.y - v1.y)));
    if (nearestPoint.distanceTo(obstical_Pos) < obstical_Radius && pointLineCollision(nearestPoint, v1, v2)) {
        return true;
    }
    return false;
}

int num = 0;

void setup(){
  size(1000,1000);
  surface.setTitle("Inverse Kinematics [CSCI 5611 Example]");
}

//Obstical
Vec2 obstical_Pos = new Vec2(500, 500);
float obstical_Radius = 50;

//Root
Vec2 root = new Vec2(500,800);

//Upper Arm
float l0 = 200; 
float a0 = 0.3; //Shoulder joint

//Lower Arm
float l1 = 200;
float a1 = 0.3; //connecting joint

//left first link
float l2 = 150;
float a2 = 0.3; //Wrist joint

Vec2 start_l1,start_l2, endPoint;

int dir = 1;

float old_a0, old_a1, old_a2;

void keyPressed(){
  if (keyCode == LEFT) root.x -= 5;
  if (keyCode == RIGHT) root.x += 5;
  if (keyCode == UP) root.y -= 5; 
  if (keyCode == DOWN) root.y += 5;
}

void solve(){  
  Vec2 goal = new Vec2(mouseX, mouseY);
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  
  old_a0 = a0;
  old_a1 = a1;
  old_a2 = a2;
  //Update wrist joint
  startToGoal = goal.minus(start_l2);
  startToEndEffector = endPoint.minus(start_l2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  do {
    a2 = old_a2;
    if (cross(startToGoal,startToEndEffector) < 0)
      a2 += angleDiff;
    else
      a2 -= angleDiff;
    fk();
    angleDiff *= 0.5;
  } while (lineBallCollision(start_l2, endPoint));
  
  //Update elbow joint
  startToGoal = goal.minus(start_l1);
  startToEndEffector = endPoint.minus(start_l1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  do {
    a1 = old_a1;
    if (cross(startToGoal,startToEndEffector) < 0)
      a1 += angleDiff;
    else
      a1 -= angleDiff;
    fk();
    angleDiff *= 0.5;
  } while (lineBallCollision(start_l2, endPoint) || lineBallCollision(start_l1, start_l2));
    
  //Update shoulder joint
  startToGoal = goal.minus(root);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPoint.minus(root);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  do {
    a0 = old_a0;
    if (cross(startToGoal,startToEndEffector) < 0)
      a0 += angleDiff;
    else
      a0 -= angleDiff;
    fk();
    angleDiff *= 0.5;
  } while (lineBallCollision(start_l2, endPoint) || lineBallCollision(start_l1, start_l2) || lineBallCollision(root, start_l1));
  
 
  //println("Angle 0:",a0,"Angle 1:",a1,"Angle 2:",a2);
}

void fk(){
  start_l1 = new Vec2(cos(a0)*l0,sin(a0)*l0).plus(root);
  start_l2 = new Vec2(cos(a0+a1)*l1,sin(a0+a1)*l1).plus(start_l1);
  endPoint = new Vec2(cos(a0+a1+a2)*l2,sin(a0+a1+a2)*l2).plus(start_l2);
}

float armW = 20;
void draw(){
  fk();
  solve();
  
  background(250,250,250);
  
  fill(150, 0, 150);
  pushMatrix();
  translate(root.x, root.y);
  rect(-37.5, -20, 75, 75);
  popMatrix();
  
  fill(214,168,133);
  pushMatrix();
  translate(root.x,root.y);
  rotate(a0);
  rect(0, -armW/2, l0, armW);
  popMatrix();
  
  
  pushMatrix();
  translate(start_l1.x,start_l1.y);
  rotate(a0+a1);
  rect(0, -armW/2, l1, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l2.x,start_l2.y);
  rotate(a0+a1+a2);
  rect(0, -armW/2, l2, armW);
  popMatrix();
  
  fill(0, 155, 0);
  circle(obstical_Pos.x, obstical_Pos.y, obstical_Radius * 2 - armW);
}


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
