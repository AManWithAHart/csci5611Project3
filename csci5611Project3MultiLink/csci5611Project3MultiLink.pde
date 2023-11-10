void setup(){
  size(1000,1000);
  surface.setTitle("Inverse Kinematics [CSCI 5611 Example]");
  rightJ = true;
  leftJ = true;
  aSpeed = true;
  println("left blue, right red");
}

//Root
Vec2 root = new Vec2(500,800);

//Matt, if you want tot fix the arm tweek, remove the toggle feature for arm joints

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





//Obstical
Vec2 obstical_Pos = new Vec2(500, 500);
float obstical_Radius = 50;

//******************************************

//SHARED BODY

//Lower Body
float b0 = 125; 
float a0 = 0.3; //Shoulder joint


//Upper Body
float b1 = 120;
float a1 = 0.3; //connecting joint

//******************************************

//LEFT LINKS

//left first link
float l1 = 150;
float leftAngle1 = 0.3; //left upper arm

//left second link
float l2 = 150;
float leftAngle2 = 0.3; //left lower arm


//left third link
float l3 = 100;
float leftAngle3 = 0.3; //left wrist

//******************************************

//RIGHT LINKS


//right first link
float r1 = 150;
float rightAngle1 = 0.3; //right upper arm


//right second link
float r2 = 150;
float rightAngle2 = 0.3; //right lower arm

//right third link
float r3 = 100;
float rightAngle3 = 0.3; //right wrist

Vec2 target = new Vec2(0,0);
float targetSpeed = 5;






Vec2 start_b1, start_b2, start_l1, start_l2, start_l3, start_r1, endPoint, endPoint2, start_r2, start_r3;

int dir = 1;
void solve(){
  
  if(root.x > 900){
    dir = -1;
  }
  else if(root.x < 100){
    dir = 1;
  }
    root.x += dir;
  
  Vec2 mover = new Vec2(0,0);
  if(leftPressed){mover = new Vec2(-targetSpeed, 0);}
  if(rightPressed){mover = new Vec2(targetSpeed, 0);}
  if(downPressed){mover = new Vec2(0, targetSpeed);}
  if(upPressed){mover = new Vec2(0, -targetSpeed);}
  target.add(mover);
  
  
  if(target.x > width - 10){
    target.x = width - 10;  
  }
  if(target.x < 10){
    target.x = 10;  
  }
  
  if(target.y < 10){
    target.y = 10;
  }
  
  if(target.y > height - 10){
    target.y = height - 10;
  }
  
  
  
  
  
  
  Vec2 goal = new Vec2(mouseX, mouseY);
  Vec2 goal2 = target;
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  
 
  
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  //************************************************************
  
  //RIGHT ARM
  
  //THIRD RIGHT LINK
  startToGoal = goal2.minus(start_r3);
  startToEndEffector = endPoint2.minus(start_r3);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  
  if(aSpeed){
    if(angleDiff > 0.05){angleDiff = 0.05;}
    if(angleDiff < -0.05){angleDiff = -0.05;}
  }
  
  
  if (cross(startToGoal,startToEndEffector) < 0)
    rightAngle3 += angleDiff;
  else
    rightAngle3 -= angleDiff;
  /*TODO: Wrist joint limits here*/
  
  if(rightJ){
    if(rightAngle3 > 1){rightAngle3 = 1;}
    if(rightAngle3 < -1){rightAngle3 = -1;}
  }
  
  fk();
  
  //SECOND RIGHT LINK
  startToGoal = goal2.minus(start_r2);
  startToEndEffector = endPoint2.minus(start_r2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  
  if(aSpeed){
    if(angleDiff > 0.05){angleDiff = 0.05;}
    if(angleDiff < -0.05){angleDiff = -0.05;}
  }
  
  
  if (cross(startToGoal,startToEndEffector) < 0)
    rightAngle2 += angleDiff;
  else
    rightAngle2 -= angleDiff;
  /*TODO: Wrist joint limits here*/
  
  if(rightJ){
    if(rightAngle2 > 1){rightAngle2 = 1;}
    if(rightAngle2 < -1){rightAngle2 = -1;}
  }
  
  fk();
  
  
  //FIRST RIGHT LINK
  startToGoal = goal2.minus(start_b2);
  startToEndEffector = endPoint2.minus(start_b2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  
  
  if(aSpeed){
    if(angleDiff > 0.05){angleDiff = 0.05;}
    if(angleDiff < -0.05){angleDiff = -0.05;}
  }
  
  if (cross(startToGoal,startToEndEffector) < 0)
    rightAngle1 += angleDiff;
  else
    rightAngle1 -= angleDiff;
  /*TODO: Wrist joint limits here*/
  
  if(rightJ){
    if(rightAngle1 > 1){rightAngle1 = 1;}
    if(rightAngle1 < -1){rightAngle1 = -1;}
  }
  
  fk();
  
  //************************************************************
  
  ////LEFT ARM
  
  //THIRD LEFT LINK
  startToGoal = goal.minus(start_l3);
  startToEndEffector = endPoint.minus(start_l3);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  
  if(aSpeed){
     if(angleDiff > 0.05){angleDiff = 0.05;}
     if(angleDiff < -0.05){angleDiff = -0.05;}
  }
  
  
  
  if (cross(startToGoal,startToEndEffector) < 0)
    leftAngle3 += angleDiff;
  else
    leftAngle3 -= angleDiff;
  /*TODO: Wrist joint limits here*/
  if(leftJ){
    if(leftAngle3 > 1){leftAngle3 = 1;}
    if(leftAngle3 < -1){leftAngle3 = -1;}
  }
  
  fk();
  
  
  //SECOND LEFT LINK
  startToGoal = goal.minus(start_l2);
  startToEndEffector = endPoint.minus(start_l2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  
  if(aSpeed){
    if(angleDiff > 0.05){angleDiff = 0.05;}
    if(angleDiff < -0.05){angleDiff = -0.05;}
  }


  if (cross(startToGoal,startToEndEffector) < 0)
    leftAngle2 += angleDiff;
  else
    leftAngle2 -= angleDiff;
  /*TODO: Wrist joint limits here*/
  
  if(leftJ){
    if(leftAngle2 > 1){leftAngle2 = 1;}
    if(leftAngle2 < -1){leftAngle2 = -1;}
  }
  
  fk();
  
  //FIRST LEFT LINK
  startToGoal = goal.minus(start_b2);
  startToEndEffector = endPoint.minus(start_b2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  
  if(aSpeed){
    if(angleDiff > 0.05){angleDiff = 0.05;}
    if(angleDiff < -0.05){angleDiff = -0.05;}
  }
  
  if (cross(startToGoal,startToEndEffector) < 0)
    leftAngle1 += angleDiff;
  else
    leftAngle1 -= angleDiff;
  /*TODO: Wrist joint limits here*/
  
  if(leftJ){
    if(leftAngle1 > 1){leftAngle1 = 1;}
    if(leftAngle1 < -1){leftAngle1 = -1;}
  }
  
  fk();
  
  
  //************************************************************
  //MAIN BODY
  
  
  //Middle Connector
  if(start_b1.distanceTo(goal) < start_b1.distanceTo(goal2)){
    startToGoal = goal.minus(start_b1);
    startToEndEffector = endPoint.minus(start_b1);
  }
  else{
    startToGoal = goal2.minus(start_b1);
    startToEndEffector = endPoint2.minus(start_b1);
  }
  
  
  
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  
  if(aSpeed){
    if(angleDiff > 0.05){angleDiff = 0.05;}
    if(angleDiff < -0.05){angleDiff = -0.05;}
  }
  if (cross(startToGoal,startToEndEffector) < 0)
    a1 += angleDiff;
  else
    a1 -= angleDiff;
   
  if(a1 > 1){a1 = 1;}
  if(a1 < -1){a1 = -1;}
    
    
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  
  //root to middle connector
  startToGoal = goal.minus(root);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPoint2.minus(root);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  
  if(aSpeed){
    if(angleDiff > 0.05){angleDiff = 0.05;}
    if(angleDiff < -0.05){angleDiff = -0.05;}
  }
  
  if (cross(startToGoal,startToEndEffector) < 0)
    a0 += angleDiff;
  else
    a0 -= angleDiff;
    
  if(a0 > -0.7){a0 = -0.7;}
  if(a0 < -2.5){a0 = -2.5;}
  
  
  fk(); //Update link positions with fk (e.g. end effector changed)
 
  //println("Angle 0:",a0,"Angle 1:",a1,"Angle 2:",leftAngle1, "Angle 3:", rightAngle1, "Angle 4:", rightAngle2);
  
  //println("RA1:",rightAngle1, "RA2:", rightAngle2, "RA3:", rightAngle3);
  //println("LA1:",leftAngle1, "LA2:", leftAngle2, "LA3:", leftAngle3);
}











void fk(){
  
  
  start_b1 = new Vec2(cos(a0)*b0,sin(a0)*b0).plus(root);
  start_b2 = new Vec2(cos(a0+a1)*b1,sin(a0+a1)*b1).plus(start_b1);

  //start_l1 = new Vec2(cos(a0+a1)*b1,sin(a0+a1)*b1).plus(start_b1);
  start_l2 = new Vec2(cos(a0+a1+leftAngle1)*l1,sin(a0+a1+leftAngle1)*l1).plus(start_b2);
  start_l3 = new Vec2(cos(a0+a1+leftAngle1+leftAngle2)*l2,sin(a0+a1+leftAngle1+leftAngle2)*l2).plus(start_l2);
  
  
  //start_r1 = new Vec2(cos(a0+a1)*b1,sin(a0+a1)*b1).plus(start_b1);
  
  start_r2 = new Vec2(cos(a0+a1+rightAngle1)*r1,sin(a0+a1+rightAngle1)*r1).plus(start_b2);
  
  start_r3 = new Vec2(cos(a0+a1+rightAngle1+rightAngle2)*r2,sin(a0+a1+rightAngle1+rightAngle2)*r2).plus(start_r2);
  
  endPoint = new Vec2(cos(a0+a1+leftAngle1+leftAngle2+leftAngle3)*l3,sin(a0+a1+leftAngle1+leftAngle2)*l3).plus(start_l3);
  
  endPoint2 = new Vec2(cos(a0+a1+rightAngle1+rightAngle2+rightAngle3)*r3,sin(a0+a1+rightAngle1+rightAngle2+rightAngle3)*r3).plus(start_r3);
  
}


boolean leftPressed, rightPressed, upPressed, downPressed, shiftPressed, paused, leftJ, aSpeed, rightJ;


void keyPressed(){
  if (keyCode == LEFT) leftPressed = true;
  if (keyCode == RIGHT) rightPressed = true;
  if (keyCode == UP) upPressed = true; 
  if (keyCode == DOWN) downPressed = true;
  if (keyCode == SHIFT) shiftPressed = true;
  if (key == 'r') paused = !paused;
  if (key == 'l') {leftJ = !leftJ; println("left arm joints:", leftJ);}
  if (key == 'k') {rightJ = !rightJ; println("right arm joints:", rightJ);}
  
  if (key == 'w') {aSpeed = !aSpeed; println("angle speed:", aSpeed);}
  
}

void keyReleased(){
  if (keyCode == LEFT) leftPressed = false;
  if (keyCode == RIGHT) rightPressed = false;
  if (keyCode == UP) upPressed = false; 
  if (keyCode == DOWN) downPressed = false;
  if (keyCode == SHIFT) shiftPressed = false;
}











float armW = 20;
void draw(){
  
  if(!paused){
    fk();
    solve();
  
    background(250,250,250);
  
  


    fill(214,168,133);
    pushMatrix();
    translate(root.x,root.y);
    rotate(a0);
    rect(0, -armW/2, b0, armW);
    popMatrix();
  
    fill(150, 0, 150);
    pushMatrix();
    translate(root.x, root.y);
    rect(-37.5, -20, 75, 75);
    popMatrix();
  
    fill(214,168,133);
  
  
    pushMatrix();
    translate(start_b1.x,start_b1.y);
    rotate(a0+a1);
    rect(0, -armW/2, b1, armW);
    popMatrix();
  
  
    fill(0,0,255);
  
    pushMatrix();
    translate(start_b2.x,start_b2.y);
    rotate(a0+a1+leftAngle1);
    rect(0, -armW/2, l1, armW);
    popMatrix();
  
    pushMatrix();
    translate(start_l2.x,start_l2.y);
    rotate(a0+a1+leftAngle1+leftAngle2);
    rect(0, -armW/2, l2, armW);
    popMatrix();
  
  
    pushMatrix();
    translate(start_l3.x,start_l3.y);
    rotate(a0+a1+leftAngle1+leftAngle2+leftAngle3);
    rect(0, -armW/2, l3, armW);
    popMatrix();
  
  
    fill(255,0,0);
  
    pushMatrix();
    translate(start_b2.x,start_b2.y);
    rotate(a0+a1+rightAngle1);
    rect(0, -armW/2, r1, armW);
    popMatrix();
  
  
  
    pushMatrix();
    translate(start_r2.x,start_r2.y);
    rotate(a0+a1+rightAngle1+rightAngle2);
    rect(0, -armW/2, r2, armW);
    popMatrix();
  
    pushMatrix();
    translate(start_r3.x,start_r3.y);
    rotate(a0+a1+rightAngle1+rightAngle2+rightAngle3);
    rect(0, -armW/2, r3, armW);
    popMatrix();
  
    fill(0,255,0);
    circle(target.x, target.y, 10);
    
    
    //fill(0, 155, 0);
    //circle(obstical_Pos.x, obstical_Pos.y, obstical_Radius * 2 - armW);
  
  }
  
  
  
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
