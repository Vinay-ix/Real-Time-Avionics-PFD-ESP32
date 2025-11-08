// ===== Boeing-Style PFD (Processing 4.x, Java mode) =====
// CSV per frame: roll,pitch,alt_m,vsi_mps,hdg_deg[,ias_knots]

import processing.serial.*;

// ---------- Serial ----------
Serial myPort;
String rx = "";

// ---------- Incoming (raw) ----------
float roll=0, pitch=0, alt=0, vsi=0, hdg=0, ias=0;
boolean hasIAS = false;

// ---------- Smoothed ----------
float sRoll=0, sPitch=0, sAlt=0, sVsi=0, sHdg=0, sIAS=0;
final float A_FAST = 0.25f, A_SLOW = 0.12f;

// ---------- FD commands ----------
float cmdRoll=0, cmdPitch=0;

// ---------- Layout ----------
final int W=1600, H=900;       // spacious; avoids clipping
final int CX=W/2, CY=H/2+20;
final int LEFT_TAPE_X = 190;            // IAS
final int RIGHT_ALT_X = W-200;          // ALT
final int RIGHT_VSI_X = W-110;          // VSI

// ---------- Units ----------
final float FPM = 196.850394f;          // m/s -> ft/min

// ---------- Helpers ----------
float wrap(float a){ a%=360; return (a<0)?a+360:a; }
float safe(float v){ return (Float.isNaN(v) || Float.isInfinite(v)) ? 0f : v; }
String safeIntStr(float v){ return nf(round(safe(v)),0,0); }
String safe1(float v){ return nf(safe(v),1,1); }
String safe2(float v){ return nf(safe(v),1,2); }

// =================================================================== SETUP
void setup() {
  size(W, H, P3D);                     // MUST be in first tab
  surface.setTitle("Boeing PFD");
  frameRate(60);

  println("Ports:"); println(Serial.list());
  // ⚠️ CHANGE THIS INDEX to match your ESP32 port
  int PORT_INDEX = 0;  // <-- CHANGE THIS INDEX (0/1/2/...)
  myPort = new Serial(this, Serial.list()[PORT_INDEX], 115200);
  myPort.bufferUntil('\n');
}

// =================================================================== DRAW
void draw() {
  background(12);

  // smoothing with NaN guards
  sRoll  += A_FAST*(safe(roll) - sRoll);
  sPitch += A_FAST*(safe(pitch) - sPitch);
  sAlt   += A_SLOW*(safe(alt) - sAlt);
  sVsi   += A_FAST*(safe(vsi) - sVsi);
  sHdg   += A_SLOW*(wrap(safe(hdg)) - sHdg);
  sIAS   += A_SLOW*(safe(ias) - sIAS);

  // --- Attitude block (horizon + ladder) ---
  pushMatrix();
  translate(CX, CY);
  rotateZ(radians(-sRoll));
  translate(0, map(constrain(sPitch, -45, 45), -45,45, -230,230));
  drawHorizon();
  drawPitchLadder();
  popMatrix();

  // --- Fixed overlays ---
  drawBankScale(sRoll);
  drawAircraftSymbol();
  drawFlightDirector(sRoll, sPitch, cmdRoll, cmdPitch);

  // --- Tapes ---
  drawIASTape(LEFT_TAPE_X, sIAS, hasIAS);
  drawAltitudeTape(RIGHT_ALT_X, sAlt);
  drawVSITape(RIGHT_VSI_X, sVsi);

  // --- Heading tape ---
  drawHeadingTape(CX, H-95, sHdg);

  // --- Text strip (top-left)
  drawTopText();
}

// =================================================================== SERIAL
void serialEvent(Serial p){
  String line = p.readStringUntil('\n');
  if (line == null) return;
  line = trim(line);

  // reject lines containing letters (debug words)
  for (int i=0;i<line.length();i++){
    char c=line.charAt(i);
    if ((c>='A' && c<='Z') || (c>='a' && c<='z')) return;
  }

  String[] t = split(line, ',');
  if (t == null || t.length < 5) return;

  try{
    float v0 = float(trim(t[0]));
    float v1 = float(trim(t[1]));
    float v2 = float(trim(t[2]));
    float v3 = float(trim(t[3]));
    float v4 = float(trim(t[4]));

    if (Float.isNaN(v0) || Float.isNaN(v1) || Float.isNaN(v2) || Float.isNaN(v3) || Float.isNaN(v4)) return;

    roll = v0;
    pitch = v1;
    alt = v2;
    vsi = v3;
    hdg = v4;

    if (t.length >= 6){
      float v5 = float(trim(t[5]));
      if (!Float.isNaN(v5) && !Float.isInfinite(v5)){
        ias = v5;
        hasIAS = true;
      } else {
        hasIAS = false;
      }
    } else {
      hasIAS = false;
    }
  }catch(Exception e){
    // swallow malformed line
  }
}

// =================================================================== ELEMENTS
void drawHorizon(){
  noStroke();
  rectMode(CENTER);

  // sky/ground
  fill(58,120,210); rect(0,-240, 2600,1400);
  fill(150,85,30);  rect(0, 240, 2600,1400);

  // horizon line
  stroke(255); strokeWeight(2);
  line(-1200,0,1200,0);
}

void drawPitchLadder(){
  stroke(255); strokeWeight(2);
  textAlign(CENTER,CENTER);
  textSize(16);

  for(int a=-45;a<=45;a+=5){
    if(a==0) continue;
    float y = -map(a,-45,45,-230,230);
    int L = (a%10==0)?90:45;

    if (a%10==0){
      line(-L,y, L,y);
      // numeric boxes
      fill(0); noStroke(); rect(-L-22, y, 28, 20); rect(L+22, y, 28, 20);
      fill(255); text(abs(a), -L-22, y); text(abs(a), L+22, y);
      stroke(255);
    }else{
      // dashed minor ticks
      line(-L, y, -L+22, y);
      line(L-22, y, L, y);
    }
  }
}

void drawBankScale(float roll){
  pushMatrix();
  translate(CX, CY-250);
  noFill(); stroke(255); strokeWeight(2);

  arc(0,0, 270,270, radians(210), radians(330));
  int[] ticks = {10,20,30,45,60};
  for(int t: ticks){ bankTick(t); bankTick(-t); }

  // Chevron top
  fill(255); noStroke();
  triangle(-10,-110, 0,-135, 10,-110);

  // pointer that rotates with roll
  stroke(255); strokeWeight(3);
  rotate(radians(-roll));
  line(0,-135, 0,-112);
  popMatrix();
}

void bankTick(int deg){
  float a=radians(deg); float R1=135, R2=116;
  float x1=R1*sin(a), y1=-R1*cos(a);
  float x2=R2*sin(a), y2=-R2*cos(a);
  line(CX+x1, CY-250+y1, CX+x2, CY-250+y2);
}

void drawAircraftSymbol(){
  stroke(255); strokeWeight(6);
  line(CX-70,CY, CX+70,CY);     // wings
  strokeWeight(5);
  line(CX, CY-14, CX, CY+50);   // fuselage
}

void drawFlightDirector(float r, float p, float cmdR, float cmdP){
  float errR = cmdR - r;
  float errP = cmdP - p;

  float px = constrain(errR*2.4, -120, 120);
  float py = constrain(-errP*2.4, -120, 120);

  stroke(255,0,150); strokeWeight(6); // magenta
  line(CX-65, CY+py, CX+65, CY+py); // pitch bar
  line(CX+px, CY-44, CX+px, CY+44); // roll bar
}

void drawIASTape(int x, float ias_knots, boolean haveIAS){
  int top = CY-130, bot = CY+130;
  stroke(255); noFill(); rect(x, CY, 90, 260);

  float scale = 2.6f; // px/kt
  float base = safe(ias_knots);

  // labels
  textAlign(RIGHT, CENTER); fill(255); textSize(16);

  for (int d=-80; d<=80; d+=5){
    float v = base + d;
    float yy = CY - d*scale;
    if (yy < top-14 || yy > bot+14) continue;

    if (d%10==0) {
      line(x-38, yy, x-3, yy);
      text(safeIntStr(v), x-44, yy);
    } else {
      line(x-22, yy, x-3, yy);
    }
  }

  // current IAS window
  stroke(255); fill(0);
  rect(x-3, CY, 78, 30);
  fill(255); textAlign(CENTER, CENTER); textSize(20);
  if (haveIAS) text(safeIntStr(base), x-3, CY);
  else         text("--", x-3, CY);
}

void drawAltitudeTape(int x, float alt_m){
  int top = CY-130, bot = CY+130;
  stroke(255); noFill(); rect(x, CY, 90, 260);

  float base = safe(alt_m);
  float scale = 3.0f; // px/m
  textAlign(LEFT,CENTER); textSize(16); fill(255);

  for(int d=-180; d<=180; d+=5){
    float yy = CY - d*scale;
    if (yy < top-14 || yy > bot+14) continue;

    if (d%10==0){
      line(x-40, yy, x-3, yy);
      text(safeIntStr(base - d), x-72, yy);
    } else {
      line(x-22, yy, x-3, yy);
    }
  }

  // box
  fill(0); stroke(255);
  rect(x-38, CY, 66, 30);
  fill(255); textAlign(CENTER, CENTER); textSize(20);
  text(safeIntStr(base), x-38, CY);
}

void drawVSITape(int x, float vsi_mps){
  int top=CY-130, bot=CY+130;
  stroke(255); noFill(); rect(x, CY, 70, 260);

  float fpm = safe(vsi_mps) * FPM;
  float scale = 0.06f; // px/fpm

  textAlign(RIGHT, CENTER); textSize(13); fill(255);

  for (int v=-3000; v<=3000; v+=250){
    float yy = CY - v*scale;
    if (yy < top-14 || yy > bot+14) continue;

    if (v%500==0){
      line(x-30, yy, x-3, yy);
      if (v!=0) text(safeIntStr(abs(v)), x-34, yy);
    } else {
      line(x-16, yy, x-3, yy);
    }
  }

  // pointer
  float py = constrain(CY - fpm*scale, top+8, bot-8);
  fill(255); noStroke(); triangle(x+3,py, x+20,py-10, x+20,py+10);

  // m/s value box
  stroke(255); fill(0);
  rect(x-1, CY+82, 54, 24);
  fill(255); textAlign(CENTER,CENTER); textSize(15);
  text(safe2(vsi_mps), x+26, CY+82);
}

void drawHeadingTape(int cx, int y, float hdg_deg){
  int w = 600;
  float px=3.2; // px per degree

  stroke(255); noFill(); rect(cx, y, w, 54);
  float zeroX = cx - wrap(hdg_deg) * px;

  textAlign(CENTER,TOP); textSize(16); fill(255);

  for(int d=-180; d<=540; d+=5){
    float dd = wrap(d);
    float xx = zeroX + d*px;
    if (xx < cx-w/2-36 || xx > cx+w/2+36) continue;

    if (d%10==0){
      line(xx, y-22, xx, y+22);
      text(nf((int)dd,3), xx, y+24);
    } else {
      line(xx, y-12, xx, y+12);
    }
  }

  // center bug & numeric box
  fill(255); noStroke(); triangle(cx, y-32, cx-10, y-20, cx+10, y-20);
  stroke(255); fill(0); rect(cx-38, y-48, 76, 24);
  fill(255); textAlign(CENTER,CENTER); text(safeIntStr(wrap(hdg_deg)), cx-38, y-48);
}

void drawTopText(){
  fill(200); textAlign(LEFT,TOP); textSize(16);
  text("ROLL  "+safe1(roll)+"°", 16,16);
  text("PITCH "+safe1(pitch)+"°", 16,36);
  text("ALT   "+safe(alt)+" m",   16,56);
  text("VSI   "+safe2(vsi)+" m/s",16,76);
  if (hasIAS) text("IAS   "+safeIntStr(ias)+" kt", 16,96);
  else        text("IAS   -- kt",                16,96);
}

// =================================================================== INPUT
void keyPressed(){
  if (keyCode==LEFT)  cmdRoll  -= 2;
  if (keyCode==RIGHT) cmdRoll  += 2;
  if (keyCode==UP)    cmdPitch += 1;
  if (keyCode==DOWN)  cmdPitch -= 1;
  if (key==' ') { cmdRoll=0; cmdPitch=0; }
}
