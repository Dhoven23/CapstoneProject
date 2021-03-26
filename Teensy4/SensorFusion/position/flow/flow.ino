#include <stdlib.h>

int arr1[35][35];
int arr2[35][35];
int error = 0;

void populate()
{
    for (int i = 0; i < 35; i++)
    {
        for (int j = 0; j < 35; j++)
        {
            if (i == j)
            {
                arr1[i][j] = 1;
                arr1[i][34-j] = 1;
            }
        }
    }

    for (int i = 0; i < 35; i++)
    {
        for (int j = 0; j < 35; j++)
        {
            if (i == j-3)
            {
                arr2[i][j] = 1;
                arr2[i][34-j] = 1;
            }
        }
    }
}

void printArrays()
{
    Serial.print("Array 1 ----------------------------------------\n");
    for (int i = 0; i < 35; i++)
    {
        Serial.print("| ");
        for (int j = 0; j < 35; j++)
        {
            Serial.print(arr1[i][j]),Serial.print(' ');
        }
        Serial.print("|\n");
    }
    Serial.print("Array 2 ----------------------------------------\n");
    for (int i = 0; i < 35; i++)
    {
        Serial.print("| ");
        for (int j = 0; j < 35; j++)
        {
            Serial.print(arr2[i][j]),Serial.print(' ');
        }
        Serial.print("|\n");
    }
}

void Diff()
{
    //Serial.print("diff  ----------------------------------------\n");
    error = 0;
    for (int i = 10; i < 25; i++)
    {
        //Serial.print("| ");
        for (int j = 10; j < 25; j++)
        {
            int diff = arr1[i][j] - arr2[i][j];
            //Serial.print("%i ", diff);
            error += abs(diff);
        }
        //Serial.print("|\n");
    }
}

void ShiftX(int dir)
{

    if (dir == 1)
    {
        for (int i = 0; i < 35; i++)
        {
            for (int j = 0; j < 34; j++)
            {
                arr1[i][j] = arr1[i][j + 1];
            }
        }
        for (int i = 0; i < 35; i++)
        {
            arr1[i][34] = 0;
        }
    }
    else if (dir == -1)
    {
        for (int i = 0; i < 35; i++)
        {
            for (int j = 34; j > 0; j--)
            {
                arr1[i][j] = arr1[i][j - 1];
            }
        }
        for (int i = 0; i < 35; i++)
        {
            arr1[i][0] = 0;
        }
    }
}

void ShiftY(int dir)
{
    if(dir == 1){
    for (int i = 0; i < 34; i++)
    {
        for (int j = 0; j < 35; j++)
        {
            arr1[i][j] = arr1[i + 1][j];
        }
    }
    for (int i = 0; i < 35; i++)
    {
        arr1[34][i] = 0;
    }
    }
    else if (dir == -1)
    {
        for (int i = 34; i > 0; i--)
        {
            for (int j = 0; j < 35; j++)
            {
                arr1[i][j] = arr1[i-1][j];
            }
        }
        for (int i = 0; i < 35; i++)
        {
            arr1[0][i] = 0;
        }
    }
}

void check(int count, int UpDn, int LfRt, bool Vert, int iterX, int iterY, int iter){
    Diff();
    if((error != 0)&&(count < 10)){  
        if (iter<=count)
        {
            if(Vert)
            {
                if(UpDn > 0)
                {
                    ShiftY(1);
                    iterY++;
                }
                else
                {
                    ShiftY(-1);
                    iterY--;
                }
            }
            else
            {
                if(LfRt > 0)
                {
                    ShiftX(1);
                    iterX++;
                }
                else
                {
                    ShiftX(-1);
                    iterX--;
                }
            }  
            iter++;  
        }
        else
        {
            if(Vert)
            {
                count++;
                Vert = !Vert;
                UpDn=-UpDn;
            }
            else 
            {
                Vert = !Vert;
                LfRt=-LfRt;
            }
            iter = 0;
        }
        //Serial.print("Error: "),Serial.println(error);
        check(count,UpDn,LfRt,Vert,iterX,iterY,iter); 
    }
    else
    {
        Serial.print("\nDisplacement: = ("),Serial.print(iterX),Serial.print(','),Serial.print(iterY),Serial.println(")");
    }
}
void setup() {

 bool dir = true;

    Serial.begin(115200);
    delay(1000);
    Serial.println("populating...");
    populate();
    delay(1000);
    Serial.println("shifting...");
    ShiftX(1);
    ShiftX(1);
    ShiftX(1);
    Serial.println("Printing...");
    printArrays();
    delay(1000);
    int t = micros();
    check(1,1,1,true,0,0,0);
    Serial.println("Made it in "),Serial.print(micros()-t),Serial.print(" microseconds!");

    

}

void loop() {
  delay(10);

}
