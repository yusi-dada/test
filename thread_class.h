#pragma once
//#define __debug__

#include <ros/ros.h>

#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

//#include <assert.h>
//#include <chrono>
//#include <cstdio>
//#include <limits.h>

using namespace std;

struct myData
{
	int in[5];
	int out[2];
};


class nanoclock
{
	private:
    chrono::high_resolution_clock::time_point begin;
  public:
  	nanoclock()
  	{	begin = chrono::high_resolution_clock::now(); }
  	long int tic()
  	{
	  	auto end = chrono::high_resolution_clock::now();
  		auto nsec = chrono::duration_cast<chrono::nanoseconds>(end-begin).count();
			begin = end;
  		return(nsec);
  	}
};


class thread_class
{
	public:
		thread_class();

		// exec
		void run_exec(size_t num_thread);		// 並列タスク処理（完了待ち）

		// process
		bool run_process(float period);			// 独立実行プロセス
		bool kill_process();								// プロセス終了	
		void setProcessTask(int n);					// プロセスタスクの設定
		void setExecTask(int n);						// 並列タスクの設定
		
	private:
		mutex exec_mtx;											// スコープは個別のインスタンス
		mutex process_mtx;									// スコープは個別のインスタンス

		// exec
		int task_id;												// タスク分配用ID
		vector<myData> tasks;								// タスクデータ
		void exec(size_t i);								// 並列タスク
		bool access(int &id,								// タスクデータアクセス
		            myData &dat, 
		            bool get = true);				

		// process
		vector<int> process_task;						// プロセスタスク
		bool inAction;											// プロセス実行中
		bool killReq;												// プロセス終了リクエスト
		void process(float period);

};

void dummyFunc();	// 42949672カウントアップ（1スレッドで60[ms]程度）
