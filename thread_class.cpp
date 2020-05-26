#include "thread_class.h"


//******************************************
//
// コンストラクタ
//
//******************************************
thread_class::thread_class()
{
	cerr << "constructor" << endl;
	inAction = false;
	killReq = false;
}

//******************************************
//
// 独立実行プロセス生成
//
//******************************************
bool thread_class::run_process(float period)
{
	if(!inAction)
	{
		thread th([this,period](){this->process(period);});
		th.detach();
		return(true);
	}
	else
	{
		cerr << "already executing" << endl;
		return(false);
	}
}
bool thread_class::kill_process()
{
	if(inAction)
	{
		killReq = true;
		return(true);
	}
	else
		return(false);
}

//******************************************
//
// 独立実行プロセス
//
//******************************************
void thread_class::process(float period)
{
	inAction = true;			// 実行中
	killReq = false;			// 終了リクエストクリア
	process_task.clear();	// プロセスタスククリア

	ros::Rate loop_rate(1.0/period);

#ifdef __debug__
	nanoclock clk;
#endif
  while (ros::ok () && !killReq)
  {
  	// 処理データ取得
  	process_mtx.lock();
	  int n = process_task.size();
	  int dat;
	  if (n>0)
	  {
	  	dat = process_task[0];										// 先頭データ取得
	  	process_task.erase(process_task.begin());	// 先頭データ削除
	  }
  	process_mtx.unlock();

		// データ数表示
	  cerr << "N: " << n << "  " << endl;
  	
  	// データ処理
  	
	  
#ifdef __debug__
  	// 時間計測
  	auto nsec = clk.tic();
		cerr << "process: " << nsec*1.e-6 << "msec" << endl;
#endif
		// 次周期まで待機		
    loop_rate.sleep();
  }
  
  // 終了処理
  cerr << "process killed." << endl;
  inAction = false;		// プロセス終了
  killReq  = false;		// 終了リクエストクリア
}

//******************************************
//
// プロセスタスクの設定
//
//******************************************
void thread_class::setProcessTask(int n)
{
	lock_guard<mutex> lock(process_mtx);
	process_task.resize(n);
}

//******************************************
//
// スレッド数を指定して実行
//
//******************************************
void thread_class::run_exec(size_t num_thread)
{
	// 開始時間計測	
	nanoclock clk;

	// スレッド生成
	vector<thread> threads;
	threads.clear();
	for(size_t i=0; i<num_thread; ++i)
		threads.emplace_back( thread([this,i](){this->exec(i);}) );

	// スレッド実行
  for(auto& thread : threads)
    thread.join();

	// 完了時間計測
	auto nsec = clk.tic();  
  cerr << "------" << endl;
  cerr << "threads: " << num_thread << endl;
  cerr << "tasks  : " << task_id    << endl;
  cerr << "process time: " << nsec*1.e-6 << "msec" << endl;
}

//******************************************
//
// 実行関数
//
//******************************************
void thread_class::exec(size_t i)
{
	int id;				// 取得データID
	myData task;	// タスク
	
	while( !access(id,task,true) )
	{
		// 処理
    dummyFunc();


		// 処理データアップロード
		access(id, task, false);

#ifdef __debug__
		// report
		exec_mtx.lock();
		cerr << "thread #" << i << " process: " << id << endl;
		exec_mtx.unlock();
#endif
	}

#ifdef __debug__
	// スレッド終了
	exec_mtx.lock();
	cerr << "thread #"<< i << " was done." << endl;
	exec_mtx.unlock();
#endif
}

//******************************************
//
// 並列タスクの設定
//
//******************************************
void thread_class::setExecTask(int n)
{
	lock_guard<mutex> lock(exec_mtx);
	task_id = 0;		// データ処理用IDクリア
	tasks.resize(n);
}

//******************************************
//
// データアクセス
//
//******************************************
bool thread_class::access(int &id, myData &dat, bool get)
{
	lock_guard<mutex> lock(exec_mtx);

	if(get)
	{
		if (task_id >= tasks.size())
			return(true);		// 完了

		id  = task_id++;
		dat = tasks[id];	// 処理前データ提供
		return(false);
	}
	else						
	{
		tasks[id] = dat;	// 処理後データ受取
		return(true);
	}
}


void dummyFunc()
{
  const size_t loop = UINT_MAX / 100;
  size_t sum = 0;
  for(size_t i=0; i<loop; ++i)
    ++sum;
}

