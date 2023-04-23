#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<netinet/in.h>
#include<stdbool.h>
#include<unistd.h>
#include <termios.h>
#include<sys/stat.h>
#include "cli.h"
#include "options.h"
#include "parse_command.h"
#include "udp.h"

#define my_port 1025
#define mc_cmd_port 1060
#define fs_cmd_port 1024
#define fs_data_port 1080
#define default_ip  "192.168.1.199\0"
#define UDP 17

int main( int argc, char *argv[] )
{
	USER_DATA data;
	int len,server_len;
	char rx_buff[1024],tx_buff[1024],temp[MAX_CHARS],temp1[MAX_CHARS];
	struct sockaddr_in server_addr;
	FILE *fp;
	static termios_t previous, current;
	char *ipAddr;
	uint16_t listenPort;
	uint16_t mcCmdPort;
	uint16_t fsCmdPort;
	uint16_t dataPort;
	bool quit = false,resume_flag=0;
	struct stat st;

	get_args(argc, argv);
	if( ! userOpts.ipValid ) {
		strcpy( userOpts.ipAddr, default_ip );
	}
	if( ! userOpts.listenPortValid ) {
		userOpts.listenPort = my_port;
	}
	if( ! userOpts.mcCmdPortValid ) {
		userOpts.mcCmdPort = mc_cmd_port;
	}
	if( ! userOpts.fsCmdPortValid ) {
		userOpts.fsCmdPort = fs_cmd_port;
	}
	if( ! userOpts.dataPortValid ) {
		userOpts.dataPort = fs_data_port;
	}
	ipAddr = userOpts.ipAddr;
	listenPort = userOpts.listenPort;
	mcCmdPort = userOpts.mcCmdPort;
	fsCmdPort = userOpts.fsCmdPort;
	dataPort = userOpts.dataPort;

	if( ! openListenerPort(ipAddr, listenPort) ) {
		printf("Could not open socket.\n");
		return -1;
	}

	disable_term_canonical(&previous, &current);

	while( ! quit ) {

		memset(data.buffer,0,MAX_CHARS+1);
		cli(data.buffer);
		strcpy(temp,data.buffer);
		strcpy(temp1,data.buffer);
		parseFields(&data);
		printf("\n");
               
		//change directory
		char *str1=strtok(temp1," ");
		if(strcmp(str1,"cd")==0)
		{
			
			str1=strtok(NULL,"\n");
			if(chdir(str1)!=0)
				perror("chdir() is failed\n");
		}

		//start command to MC
		else if(isCommand(&data,"start",1)&&(resume_flag==0))
		{
			bool a;
			tx_buff[0]=1;
			tx_buff[1]=0;
			//strcpy(tx_buff,"start");
			a = sendData(ipAddr, mcCmdPort, (const char *)&tx_buff);
			if(!a)
			{
				printf("error\n");
			}
			else
			{
				printf("client sent command:%s\n",tx_buff);
				
				len = receiveData( (char *)rx_buff, sizeof(rx_buff) );
				if(len!=-1)
				{
					if(strcmp(rx_buff,"received")==0)
					{
						printf("ack received\n");
					}
				}
			}
		}

		//stop command to MC
		else if(isCommand(&data,"stop",1)&&(resume_flag==0))
		{
 			tx_buff[0]=0xB;
			tx_buff[1]=0;
			sendData(ipAddr, mcCmdPort, (const char *)&tx_buff);
			printf("client sent command:%s\n",tx_buff);
			len = receiveData( (char *)rx_buff, sizeof(rx_buff) );
			if(len!=-1)
			{
				if(strcmp(rx_buff,"received")==0)
				{
					printf("ack received\n");
				}
			}
		}

		//pause command to MC
		else if(isCommand(&data,"pause",1)&&(resume_flag==0))
		{

			tx_buff[0]=2;
			tx_buff[1]=0;
			sendData(ipAddr, mcCmdPort, (const char *)&tx_buff);
			printf("client sent command:%s\n",tx_buff);
			len = receiveData( (char *)rx_buff, sizeof(rx_buff) );
			if(len!=-1)
			{
				if(strcmp(rx_buff,"received")==0)
				{
					printf("ack received\n");
				}
			}
			resume_flag=1;
		}

		//resume command to MC
		else if(isCommand(&data,"resume",1)&&(resume_flag==1))
		{
			tx_buff[0]=0xC;
			tx_buff[1]=0;
			sendData(ipAddr, mcCmdPort, (const char *)&tx_buff);
			printf("client sent command:%s\n",tx_buff);
			len = receiveData( (char *)rx_buff, sizeof(rx_buff) );
			if(len!=-1)
			{
				if(strcmp(rx_buff,"received")==0)
				{
					printf("ack received\n");
				}
			}
			resume_flag=0;
		}


		//Calibrate command to MC
		else if(isCommand(&data,"calibrate",1)&&(resume_flag==0))
		{
			tx_buff[0]=5;
			tx_buff[1]=0;
			sendData(ipAddr, mcCmdPort, (const char *)&tx_buff);
			printf("client sent command:%s\n",tx_buff);
			len = receiveData( (char *)rx_buff, sizeof(rx_buff) );
			if(len!=-1)
			{
				if(strcmp(rx_buff,"received")==0)
				{
					printf("ack received\n");
				}
			}
		}

        else if(isCommand(&data,"home",1)&&(resume_flag==0))
        {
            tx_buff[0]=15;
            tx_buff[1]=0;
            sendData(ipAddr, mcCmdPort, (const char *)&tx_buff);
            printf("client sent command:0x%x\n",tx_buff);
            len = receiveData( (char *)rx_buff, sizeof(rx_buff) );
            if(len!=-1)
            {
                if(strcmp(rx_buff,"received")==0)
                {
                    printf("ack received\n");
                }
            }
        }

		//jog command to MC
		else if(((isCommand(&data,"jog",4))||(isCommand(&data,"jog",3))||(isCommand(&data,"jog",2)))&&(resume_flag==0))
		{
			strcpy(tx_buff,"\6");
			strcat(tx_buff," ");
			char *str=strtok(temp," ");
			str=strtok(NULL,"\n");
			strcat(tx_buff,str);
			sendData(ipAddr, mcCmdPort, (const char *)&tx_buff);
			printf("client sent command:%s\n",tx_buff);
			len = receiveData( (char *)rx_buff, sizeof(rx_buff) );
			if(len!=-1)
			{
				if(strcmp(rx_buff,"received")==0)
				{
					printf("ack received\n");
				}
			}
		}

		//SBT command to MC
		else if(isCommand(&data,"sbt",2)&&(resume_flag==0))
		{
			strcpy(tx_buff,"\xE");
			strcat(tx_buff," ");
			char *str=strtok(temp," ");
			str=strtok(NULL,"\n");
			strcat(tx_buff,str);
			sendData(ipAddr, mcCmdPort, (const char *)&tx_buff);
			printf("client sent command:%s\n",tx_buff);
			len = receiveData( (char *)rx_buff, sizeof(rx_buff) );
			if(len!=-1)
			{
				if(strcmp(rx_buff,"received")==0)
				{
					printf("ack received\n");
				}
			}
		}

		//arcto command to MC
		else if(isCommand(&data,"arcto",4)&&(resume_flag==0))
		{
			strcpy(tx_buff,"\xD");
			strcat(tx_buff," ");
			char *str=strtok(temp," ");
			str=strtok(NULL,"\n");
			strcat(tx_buff,str);
			sendData(ipAddr, mcCmdPort, (const char *)&tx_buff);
			printf("client sent command:%s\n",tx_buff);
			len = receiveData( (char *)rx_buff, sizeof(rx_buff) );
			if(len!=-1)
			{
				if(strcmp(rx_buff,"received")==0)
				{
					printf("ack received\n");
				}
			}
		}


		//Status Command to MC and FS
		else if(isCommand(&data,"status",2)&&(resume_flag==0))
		{
			char *str=strtok(temp," ");
			str=strtok(NULL,"\n");
			//This command will be sent to MC
			if(strcmp(str,"-c")==0)
			{
				tx_buff[0]=7;
				tx_buff[1]=0;
				sendData(ipAddr, mcCmdPort, (const char *)&tx_buff);
				printf("client sent command:%s\n",tx_buff);
				len = receiveData( (char *)rx_buff, sizeof(rx_buff) );
				if(len!=-1)
				{
					printf("Motion Control returns the last known coordinates of the router: %s\n",rx_buff);
				}
			}
			//This command will be sent to FS
			else if(strcmp(str,"-p")==0)
			{
				tx_buff[0]=8;
				tx_buff[1]=0;
				sendData(ipAddr, fsCmdPort, (const char *)&tx_buff);
				printf("client sent command:%s\n",tx_buff);
				len = receiveData( (char *)rx_buff, sizeof(rx_buff) );
				if(len!=-1)
				{
					printf("file system returns the file completion percentage: %s",rx_buff);
					printf("%c\n",'%');
				}
			}
		}

		// Upload file is sent to FS
		else if(isCommand(&data,"upload",2)&&(resume_flag==0))
		{

			char *str=strtok(temp," ");
			str=strtok(NULL,"\n");
			strcpy(tx_buff,"\x9");
			strcat(tx_buff," ");
			fp=fopen(str,"r");
			if(fp == NULL)
			{
				printf("unable to open file\n");
			}
			else
			{
				stat(str,&st);
				int file_size=st.st_size;
				char file[500];
				sprintf(file,"%s %d",str,file_size);
				strcat(tx_buff,file);
				sendData(ipAddr, fsCmdPort, (const char *)&tx_buff);
				printf("client sent command:%s\n",tx_buff);
				len = receiveData( (char *)rx_buff, sizeof(rx_buff) );
				if(len!=-1)
				{
					
					if(strcmp(rx_buff,"received")==0)
					{
						printf("file opened sucessfully");
						fflush(stdout);
						server_addr.sin_port=htons(dataPort);
						server_len=sizeof(server_addr);

						while(fread(&tx_buff,sizeof(char),512,fp))
						{
							sendData(ipAddr, dataPort, (const char *)&tx_buff);
							len = receiveData( (char *)rx_buff, sizeof(rx_buff) );
							if(len!=-1)
							{
								if(strncmp(rx_buff,"received",8)==0)
								{
									 printf("acknowledgement received from server that it has received data please send next packet\n");
								}
								else
									printf("received wrong acknowledgment");
							}
							memset(rx_buff,0,strlen(rx_buff));
							memset(tx_buff,0,strlen(tx_buff));
						}
					}


				}
				fclose(fp);
			}

		}



		// setactive file is sent to FS
		else if(isCommand(&data,"setactive",2)&&(resume_flag==0))
		{
			char *str=strtok(temp," ");
			str=strtok(NULL,"\n");
			strcpy(tx_buff,"\x3");
			strcat(tx_buff," ");
			strcat(tx_buff,str);
			sendData(ipAddr, fsCmdPort, (const char *)&tx_buff);
			printf("client sent command:%s\n",tx_buff);
			len = receiveData( (char *)rx_buff, sizeof(rx_buff) );
			if(len!=-1)
			{
				if(strcmp(rx_buff,"received")==0)
				{
					printf("ack received for setactive file\n");
				}
			}
		}

		else if(isCommand(&data,"dir",1)&&(resume_flag==0))
		{
			tx_buff[0]=0xA;
			tx_buff[1]=0;
			sendData(ipAddr, fsCmdPort, (const char *)&tx_buff);
			printf("client sent command:%s\n",tx_buff);
			len = receiveData( (char *)rx_buff, sizeof(rx_buff) );
			if(len!=-1)
			{
				printf("dir :%s\n",rx_buff);
			}
		}

		//Local commands passed directly to Shell
		else if(isCommand(&data,"ls",1))
		{
			system(temp);
		}
		else if(isCommand(&data,"pwd",1))
		{
			getcwd(temp,sizeof(temp));
			printf("pwd: %s\n",temp);
		}
		else if(isCommand(&data,"quit",1))
		{
			quit = true;
		}
		else if(! strcmp(&data.buffer,"help") )
		{
			if( data.fieldCount > 1 ) {
				usage(getFieldString(&data, 1));
			} else {
				usage(NULL);
			}
		}
		else
		{
			printf("inavalid command\n");
			printf("Please enter help to know valid commands\n");
		}
		memset(rx_buff,0,strlen(rx_buff));
		memset(tx_buff,0,strlen(tx_buff));
		printf("\n");
	}
	closeListenerPort();
	restore_term(&previous);
}
