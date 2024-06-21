#define LPWM 5
#define LABIN 7
#define LBBIN 8
#define RPWM 6
#define RABIN 9
#define RBBIN 4



void setup() {
  // put your setup code here, to run once:
  Serial3.begin(9600);
  Serial.begin(9600);
}

double lk = 1;
double rk = 1;

void go(int ml, int mr){
  ml *= lk;
  mr *= rk;
  ml = constrain(ml, -255, 255);
  mr = constrain(mr, -255, 255);
  if(ml < 0){
    digitalWrite(LABIN, HIGH);
    digitalWrite(LBBIN, LOW);
  }else{
    digitalWrite(LABIN, LOW);
    digitalWrite(LBBIN, HIGH);  
  }
  analogWrite(LPWM, abs(ml));
  if(mr < 0){
    digitalWrite(RABIN, HIGH);
    digitalWrite(RBBIN, LOW);
  }else{
    digitalWrite(RABIN, LOW);
    digitalWrite(RBBIN, HIGH);  
  }
  analogWrite(RPWM, abs(mr));
}

int a[2] = {0, 0};
int i = 0;
void loop() {
  // put your main code here, to run repeatedly:
  
  while(Serial3.available()){
      int k = Serial3.parseInt();
      
      if (k==0){
        continue;
        }
      a[i] = k;
      i = (i + 1) % 2;
      Serial.println(a[i]);
  }

 
  go(a[0], a[1]);
}
