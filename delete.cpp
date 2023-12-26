// #include <iostream>
// #include <unistd.h>
// #include <sys/stat.h>
// #include <fcntl.h>

// #include <iostream>
// #include <sys/types.h>
// #include <unistd.h>
// using namespace std;
// int main ()
// {
    //--------------------read from one file and write is content to another file-----------------------//
    // int n,fd,fdd,m;
    // char buff[100];  
    // char bufff[100]; 
    // fd = open ("source.txt",O_RDONLY);
    // if (fd==-1) cout<<"fdd failed to open\n";
 
    // n = read(fd,buff,100);

    // write(1,buff,n);
    // fdd = open ("destination.txt",O_CREAT|O_RDWR,0777);
    // if (fdd==-1) cout<<"fdd failed to open\n";
    // write(fdd,buff,n);
    // m=read(fdd,bufff,100);
    // cout<<"\nContent in desination is:\n";
    // write(1,bufff,m);
   
    // close(fd);
    // close(fdd);
    //-----------------------------------------------------------------------------------------------------//
    // int n,fd;
    // char buff[100];  

    // fd = open ("config.txt",O_CREAT|O_RDWR,0777);
    // if (fd==-1) cout<<"fd failed to open\n";
    // //n = read(fd,buff,100);
    // write(1,"Enter text to write.\n",25);
    // n = read(0,buff,100);
    // cout<<"content:\n";
    // write(1,buff,n);
    // write(fd,buff,n);
    // close(fd);
    //.........................................................

    // int n,fd,fdd,m;
    // char buff[100];  
    // char bufff[100]; 
    // fd = open ("config.txt",O_RDONLY);
    // if (fd==-1) cout<<"fdd failed to open\n";
 
    // n = read(fd,buff,100);

    // write(1,buff,n);
    // cout<<"\nwords starting with vowels are:\n";
    // int temp=0;
    // int chk;
    // int flag=1;
    // while(temp<n){
    //     if(flag==1){
    //         if(buff[temp]=='a' ||buff[temp]=='e'||buff[temp]=='i'||buff[temp]=='o'||buff[temp]=='u'||buff[temp]=='A'||buff[temp]=='E'||buff[temp]=='I'||buff[temp]=='O'||buff[temp]=='U'){
    //             while(buff[temp]!=' '){
    //                 cout<<buff[temp];
    //                 temp++; 
    //             }  
    //         }
    //         flag=0;
    //     }

    //     if(buff[temp]==' '){
    //         cout<<" ";
    //         flag=1;
    //     }
    //     temp++;
    // }
    // close(fd);
    //.......................................................

    // int n,fd,fdd,m;
    // char buff[100];  
    // char bufff[100]; 
    // fd = open ("News.txt",O_RDONLY);
    // if (fd==-1) cout<<"fdd failed to open\n";
 
    // n = read(fd,buff,100);

    // write(1,buff,n);
    // cout<<"\nwords starting with vowels are:\n";
    // int temp=0;
    // int count=0;
    // int flag=1;
    // while(temp<n){
    //     if(flag==1){
    //         if(buff[temp]=='a' && buff[temp+1]=='n'&& buff[temp+2]=='d'){
    //             while(buff[temp]!=' '){
    //                 write(1,&buff[temp],1);
    //                 temp++; 
    //             }
    //             write(1,&buff[temp+1],1);
    //             count++;  
    //         }
    //         flag=0;
    //     }
    //     if(buff[temp]==' '){
    //         //cout<<" ";
    //         flag=1;
    //     }
    //     temp++;
    // }
    // cout<<"\nTotal 'and' phrase count = "<<count<<endl;
    // close(fd);
    //.................................................................

    // int n,fd,fdd,m;
    // char buff[100];  
    // char bufff[100]; 
    // fd = open ("News.txt",O_RDONLY);
    // if (fd==-1) cout<<"fdd failed to open\n";
 
    // n = read(fd,buff,100);

    // write(1,buff,n);
    // //cout<<"\nwords starting with vowels are:\n";
    // int temp=0;
    // int count_s=0;
    // int count_n=0;
    // int flag=1;
    // while(temp<n){
    //     if(buff[temp]== ' '){
    //         count_s++;
    //     }
    //     else if(buff[temp]== '\n'){
    //         count_n++;
    //     }

    //     temp++;
    // }
    // cout<<"\nTotal spaces are = "<<count_s<<endl;
    // cout<<"Total newline characters are = "<<count_n<<endl;
    // close(fd);
    //..............................................................................

    // int pid = fork();
    // int pid2,pid3;
    // if(pid==0)
    // {
    //     cout << "I am a child process 1 and my PID is : " << getpid();
    //     cout << endl;
    //     sleep(10); 
    // }
    // else if (pid > 0)
    // {
    //     cout << "I am a parent process and my PID is: " << getpid();
    //     cout << endl; 

    //     pid2 = fork();
    //     if(pid2==0)
    //     {
    //         cout << "I am a child process 2 and my PID is : " << getpid();
    //         cout << endl; 
    //         sleep(5);
    //     }
    //     else if(pid2>0){
    //         cout << "I am a parent process and my PID is: " << getpid();
    //         cout << endl; 
    //         cout<<waitpid(pid2, NULL, 0)<<endl;
    //         pid3 = fork();
    //         if(pid3==0)
    //         {
    //             cout << "I am a child process 3 and my PID is : " << getpid();
    //             cout << endl;
    //             sleep(8); 
    //         }
    //         else if(pid3>0){
    //             cout << "I am a parent process and my PID is: " << getpid();
    //             cout << endl; 
    //             wait(NULL);
    //         }
    //     }
    //     int cpid = waitpid(pid2,NULL,0);
        
    // }
    // //int cpid = waitpid(pid2, NULL,0);
    // else
    // {
    //     cout << "Fork Failed";
    //     cout << endl; 
    // }

//     struct items {
//         string name;
//         bool fly;

//     };

    
//     return 0;


// }

#include <iostream>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include<pthread.h>
#include<stdio.h>
using namespace std;

void *  factorial (void * p){
    int n= *(int *) p;
    cout<<"n = "<<n<<endl;
    if(n<1){
        cout<<"invalid number";
        return 0;
    }
    int f=n;
    cout<<"Fictorial of "<<n<<"! = ";
    int i;
    for ( i = n; i >1; i--)

    {
        cout<<i<<" x ";
        f=f*(i-1);

        /* code */
    }
    cout<<i<<" = "<<f<<endl;
    pthread_exit(NULL);
    
}
void *  sum (void * p){
    int n= *(int *)p;
    cout<<"n = "<<n<<endl;

    int s=0;
    for (int i = 0; i <= n; i++)
    {
        cout<< i <<" + ";
        s=s+i;
        /* code */
    }
    cout<<" = "<<s;
    pthread_exit(NULL);

    
}




int main(){

    pthread_t kid,kid2;
    int n=4;
    pthread_create(&kid, NULL, factorial, (void*)&n);
    pthread_create(&kid2, NULL, sum, (void*)&n);
    pthread_join(kid, NULL);
    pthread_join(kid2, NULL);
    cout<<"\nno more thread.....\n";



}


