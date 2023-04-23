/*
 * parse_command.c
 *
 *  Created on: Apr 22, 2022
 *      Author: dmainz copied Uday's code in here to make it modular
 */
#include "parse_command.h"
#include <stdbool.h>

char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
   int i=0;
   for(i=0;i<=data->fieldCount;i++)
   {
       if(i==(fieldNumber))
        return (char*) &(data->buffer[data->fieldPosition[i]]);
   }

       return 0;
}
int32_t myatoi(char* str)
{
    int32_t r=0,i;
    for(i=0;str[i]!='\0';i++)
        r=r*10+str[i]-'0';
    return r;

}
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    int i=0,k;
       for(i=0;i<=data->fieldCount;i++)
       {
           if(i==(fieldNumber))
           {

              k= myatoi(&data->buffer[data->fieldPosition[i]]);
            return  k;
           }
       }

           return 0;
}
/* */
bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
  uint8_t i=0,flag=0;
  if(minArguments<=data->fieldCount)
  {
    while((strCommand[i]!='\0')||(data->buffer[i]!='\0'))
    {
        if(strCommand[i]!=data->buffer[i])
        {
            flag=1;
            break;
        }
        i++;
      }
      if(flag==0)
      {
       return(1);
      }
      else
      {
       return(0);
      }
  }
  else
    return(0);
 }

void parseFields(struct _USER_DATA *data)
{
    uint8_t i=0,j=0,del=1,alpha=0,num=0;
    data->fieldCount=0;
    for(i=0;data->buffer[i]!='\0';i++)
    {
      if(data->fieldCount!=MAX_FIELDS)
      {
        if((data->buffer[i]>=65 && data->buffer[i]<=90)||(data->buffer[i]>=97 && data->buffer[i]<=122)&&(del==1))
        {
            data->fieldType[j] ='a';
            data->fieldPosition[j++]=i;
            alpha=1;
            del=0;
            data->fieldCount=data->fieldCount+1;
        }
        else if((data->buffer[i]>=48 && data->buffer[i]<=57)&&(del==1))
        {
            data->fieldType[j] ='n';
            data->fieldPosition[j++]=i;
            num=1;
            del=0;
            data->fieldCount=data->fieldCount+1;
         }
        else
        {
            if((data->buffer[i]>=65 && data->buffer[i]<=90)||(data->buffer[i]>=97 && data->buffer[i]<=122)||(data->buffer[i]==46)|| (data->buffer[i]==47))
            {

            }
            else if((data->buffer[i]>=48 && data->buffer[i]<=57)||(data->buffer[i]==46)|| (data->buffer[i]==47))
            {

            }
            else
            {
                del=1;
                alpha=0;
                num=0;
            }
        }
     }
  }
  for(i=0;data->buffer[i]!='\0';i++)
  {
    if((data->buffer[i]>=65 && data->buffer[i]<=90)||(data->buffer[i]>=97 && data->buffer[i]<=122)||(data->buffer[i]>=48 && data->buffer[i]<=57))
    {

    }
    else
    {
     data->buffer[i] = '\0';
    }
  }
}
//parse field end

