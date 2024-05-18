#include<stdio.h>
#include<time.h>
#include<stdlib.h>
#include <assert.h>

// CUDA runtime
#include <cuda_runtime.h>
#include <cuda_profiler_api.h>

// Helper functions and utilities to work with CUDA

#define BS 1024
#define ROAD 0
#define WIDTH 8 
#define HEIGHT  8 
#define MAPSIZE (WIDTH*HEIGHT) 
#define WALL (MAPSIZE*16 -1)
//#define N  524288/* 配列の長さ、2の30乗 
 /* GPUカーネル関数の定義*/
 __global__ void bidirectional_search(int *Dend,int *j,int *DA,int *g,int *start,int *goal,int *end_sg)
 {
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if(i>=(((MAPSIZE)))){
    return;
	}
	if(*end_sg>0){	
		return;
	}
	if((DA[i]!=WALL)&&((i)!=(*goal))){
if(((g[i])==0)&&(g[i+1]|g[i-1]|g[i+WIDTH]|g[i-WIDTH])){
					Dend[i]=(*j);
					if(i==(*start)){
						*end_sg=1;
					}
		}
			g[i]=g[i+1]|g[i-1]|g[i+WIDTH]|g[i-WIDTH];
	
	}
return;
}
int main(void)
{
 int *i;
// int j;
 int *start_gpu,*goal_gpu;	
 int *start,*goal;	
 int *start_flag,*goal_flag;
 int *Dgoal_flag;
 int *ROUTE;
 int *A; /* ホストメモリ用のポインタ*/
 long start_time,end_time,pre_time_start,pre_time_end;
 int *ii;
 int *end,*gend;
 int *wall_end;
 int *Dwall_end;
 int j=0;
 wall_end = (int *)malloc((MAPSIZE) *sizeof(int));
     // 各行ごとに列数分のメモリを確保
  ROUTE=(int *)malloc(sizeof(int)*(MAPSIZE)); /* 配列Aの領域確保*/
  goal_flag = (int *)malloc(sizeof(int)*(MAPSIZE)); /* 配列Aの領域確保*/
  start = (int *)malloc(sizeof(int)); /* 配列Aの領域確保*/
  goal = (int *)malloc(sizeof(int)); /* 配列Aの領域確保*/
  i = (int *)malloc(sizeof(int)); /* 配列Aの領域確保*/
  start_flag = (int *)malloc(sizeof(int)*(MAPSIZE)); /* 配列Aの領域確保*/
  end = (int *)malloc(sizeof(int)); /* 配列Aの領域確保*/
  *end=0;
  *start=WIDTH+1;
  *goal=(MAPSIZE)-WIDTH-2;
  start_flag[*start]=1;
  goal_flag[*goal]=1;
  	  for(int in=0;in<(MAPSIZE);in++){
		if((in/WIDTH)==0||(in/WIDTH)==HEIGHT-1){
				ROUTE[in]=WALL;
		}else if((in%WIDTH)==0||(in%WIDTH)==WIDTH-1){
				ROUTE[in]=WALL;
		}else if((in/WIDTH)%2==1){
				ROUTE[in]=ROAD;
		}else if((in/WIDTH)%2==0){
			if((in%WIDTH)%2==0){
				ROUTE[in]=ROAD;
			}else{
				ROUTE[in]=WALL;
			}
		}
		wall_end[in]=WALL;
	}
		wall_end[*goal]=0;
 pre_time_start=clock();
 cudaMalloc((int**)&Dwall_end, sizeof(int)*MAPSIZE); 
	//printf("s%04d,g%d\n",*start,*goal);
 cudaMalloc((int**)&ii, sizeof(int)); 
 cudaMalloc((int**)&A, sizeof(int)*(MAPSIZE)); 
 cudaMalloc((int**)&Dgoal_flag, sizeof(int)*(MAPSIZE)); 
 cudaMalloc((int**)&start_gpu, sizeof(int)); 
 cudaMalloc((int**)&goal_gpu, sizeof(int)); 
 cudaMalloc((int**)&gend, sizeof(int)); 
 cudaMemcpy( start_gpu,start, sizeof(int), cudaMemcpyDefault);
 cudaMemcpy( goal_gpu,goal, sizeof(int), cudaMemcpyDefault);
 pre_time_end=clock();
	//printf("pre_time=%ld\n",pre_time_end-pre_time_start);
	for(*i=0;*i<(MAPSIZE);*i=(*i+1)){	
		//printf("%4d,",ROUTE[*i]);
		if((*i%WIDTH)==(WIDTH-1)){
			//printf("\n");
		}
	}
	start_time=clock();
	for(*i=1;(*i)<MAPSIZE;*i=(*i+1)){
 		cudaMemcpy( Dwall_end,wall_end, sizeof(int)*(MAPSIZE), cudaMemcpyDefault);
 		cudaMemcpy( ii,i, sizeof(int), cudaMemcpyDefault);
 		cudaMemcpy( A,ROUTE, sizeof(int)*(MAPSIZE), cudaMemcpyDefault);
 		cudaMemcpy( Dgoal_flag,goal_flag, sizeof(int)*(MAPSIZE), cudaMemcpyDefault);
 		cudaMemcpy( gend,end ,sizeof(int), cudaMemcpyDefault);
		bidirectional_search<<<((MAPSIZE)+BS-1)/BS, BS,2>>>(Dwall_end  ,ii,A,Dgoal_flag,start_gpu,goal_gpu,gend);
		cudaDeviceSynchronize();
 		cudaMemcpy( wall_end,Dwall_end, sizeof(int)*(MAPSIZE), cudaMemcpyDefault);
 		cudaMemcpy( ROUTE,A, sizeof(int)*(MAPSIZE), cudaMemcpyDefault);
 		cudaMemcpy( goal_flag,Dgoal_flag, sizeof(int)*(MAPSIZE), cudaMemcpyDefault);
 		cudaMemcpy( end,gend ,sizeof(int), cudaMemcpyDefault);
		//printf("end------------\n");
		if(*end==1){
				j++;
				if(j>1){
				break;
			}
}
	for(int in=0;in<(MAPSIZE);in=(in+1)){	
				if((in%WIDTH)==(WIDTH-1)){
					//printf("\n");
				}
		if(*end!=1){
		}
	}
}
		wall_end[*goal]=0;
		//printf("end------------%d\n",*i);
	end_time=clock();
	for(*i=0;*i<(MAPSIZE);*i=(*i+1)){	
		//printf("%4d,",wall_end[*i]);
		ROUTE[*i]=wall_end[*i];
		if(((*i)%WIDTH)==(WIDTH-1)){
			//printf("\n");
		}
	}
	cudaFree(A);
	cudaFree(Dwall_end);
	cudaFree(start_gpu);
	cudaFree(ii);
	cudaFree(goal_gpu);
	cudaFree(gend);
	cudaFree(Dwall_end);
	free(i);
	free(end);
	free(wall_end);
	//printf("s%04d,g%d\n",*start,*goal);
	//printf("exe_time=%ld\n,",(end_time-start_time));
	if(0){
		int nowplot,nextplot;
		int count;
		int *result;
		  result= (int *)malloc(sizeof(int)*(MAPSIZE)); /* 配列Aの領域確保*/
					nowplot=(*start);
					nextplot=(*start);
						
					count=0;
					while(nowplot!=(*goal)){
						//printf("%d ", nowplot);
						result[count]=nowplot;
						if(ROUTE[nowplot-1]!=-1){
							if(ROUTE[nextplot]>=ROUTE[nowplot-1]){
								nextplot=nowplot-1;
							}
						}
						if(ROUTE[nowplot+1]!=-1){
							if(ROUTE[nextplot]>=ROUTE[nowplot+1]){
								nextplot=nowplot+1;
							}
						}
						if(ROUTE[nowplot+WIDTH]!=-1){
							if(ROUTE[nextplot]>=ROUTE[nowplot+WIDTH]){
								nextplot=nowplot+WIDTH;
							}
						}
						if(ROUTE[nowplot-WIDTH]!=-1){
							if(ROUTE[nextplot]>=ROUTE[nowplot-WIDTH]){
								nextplot=nowplot-WIDTH;
							}
						}
						nowplot=nextplot;        
						count++;
					}
					result[count]=(*goal);
					count=0;
					while(result[count]!=(*goal)){
						//printf(" %d ",result[count]);
						count++;
					}
	}	
	free(ROUTE);
	free(start);
	free(goal);
	printf("exe_time=%ld\n,",(end_time-start_time));
	return 0;
 }
