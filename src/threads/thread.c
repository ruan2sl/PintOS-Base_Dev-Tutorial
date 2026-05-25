#include "threads/thread.h"
#include <debug.h>
#include <stddef.h>
#include <random.h>
#include <stdio.h>
#include <string.h>
#include "threads/flags.h"
#include "threads/interrupt.h"
#include "threads/intr-stubs.h"
#include "threads/palloc.h"
#include "threads/switch.h"
#include "threads/synch.h"
#include "threads/vaddr.h"
#include "threads/float.h"
#include "devices/timer.h"
#ifdef USERPROG
#include "userprog/process.h"
#endif

#define THREAD_MAGIC 0xcd6abf4b
#define A 55

static int load_avg;
static int f_59_60; 
static int f_1_60;  

static struct list ready_list;
static struct list sleep_list;
static struct list all_list;
static struct thread *idle_thread;
static struct thread *initial_thread;
static struct lock tid_lock;

struct kernel_thread_frame 
  {
    void *eip;                  
    thread_func *function;      
    void *aux;                  
  };

static long long idle_ticks;    
static long long kernel_ticks;  
static long long user_ticks;    

#define TIME_SLICE 4            
static unsigned thread_ticks;   

bool thread_mlfqs;

static void kernel_thread (thread_func *, void *aux);

static void idle (void *aux UNUSED);
static struct thread *running_thread (void);
static struct thread *next_thread_to_run (void);
static void init_thread (struct thread *, const char *name, int priority);
static bool is_thread (struct thread *) UNUSED;
static void *alloc_frame (struct thread *, size_t size);
static void schedule (void);
void thread_schedule_tail (struct thread *prev);
static tid_t allocate_tid (void);

static void mlfqs_update_load_avg (void);
static void mlfqs_update_one_recent_cpu (struct thread *t, void *aux UNUSED);
static void mlfqs_update_all_recent_cpu (void);
static void mlfqs_update_one_priority (struct thread *t, void *aux UNUSED);
static void mlfqs_update_all_priority (void);

void
thread_init (void) 
{
  ASSERT (intr_get_level () == INTR_OFF);

  lock_init (&tid_lock);
  list_init (&ready_list);
  list_init (&sleep_list); 
  list_init (&all_list);

  if (thread_mlfqs) 
    {
      load_avg = FLOAT_CONST(0);
      f_59_60 = FLOAT_DIV(FLOAT_CONST(59), FLOAT_CONST(60));
      f_1_60  = FLOAT_DIV(FLOAT_CONST(1), FLOAT_CONST(60));
    }

  initial_thread = running_thread ();
  init_thread (initial_thread, "main", PRI_DEFAULT);
  initial_thread->status = THREAD_RUNNING;
  initial_thread->tid = allocate_tid ();
}

void
thread_start (void) 
{
  struct semaphore idle_started;
  sema_init (&idle_started, 0);
  thread_create ("idle", PRI_MIN, idle, &idle_started);

  intr_enable ();
  sema_down (&idle_started);
}

void
thread_tick (void) 
{
  struct thread *t = thread_current ();

  if (t == idle_thread)
    idle_ticks++;
#ifdef USERPROG
  else if (t->pagedir != NULL)
    user_ticks++;
#endif
  else
    kernel_ticks++;

  if (++thread_ticks >= TIME_SLICE)
    intr_yield_on_return ();
}

void
thread_mlfqs_(int64_t tick) {
  struct thread *t = thread_current ();

  if (t != idle_thread) {
    t->recent_cpu = FLOAT_ADD_MIX(t->recent_cpu, 1);
  }

  if (tick % TIMER_FREQ == 0) {
    mlfqs_update_load_avg ();
    mlfqs_update_all_recent_cpu ();
  }

  /* A cada 4 ticks: atualiza prioridades, reordena e verifica preempção */
  if (tick % 4 == 0) {
    mlfqs_update_all_priority();
    if (!list_empty(&ready_list)) {
      list_sort(&ready_list, thread_compare_priority, NULL);
      struct thread *highest = list_entry(list_front(&ready_list), struct thread, elem);
      if (t->priority < highest->priority) {
        intr_yield_on_return(); /* Preempção via interrupção */
      }
    }
  }
}

void
thread_print_stats (void) 
{
  printf ("Thread: %lld idle ticks, %lld kernel ticks, %lld user ticks\n",
          idle_ticks, kernel_ticks, user_ticks);
}

tid_t
thread_create (const char *name, int priority,
               thread_func *function, void *aux) 
{
  struct thread *t;
  struct kernel_thread_frame *kf;
  struct switch_entry_frame *ef;
  struct switch_threads_frame *sf;
  tid_t tid;

  ASSERT (function != NULL);

  t = palloc_get_page (PAL_ZERO);
  if (t == NULL)
    return TID_ERROR;

  if (thread_mlfqs) init_thread(t, name, PRI_MAX);
  else init_thread (t, name, priority);

  tid = t->tid = allocate_tid ();

  if (thread_mlfqs)
    {
      struct thread *cur = thread_current ();
      t->nice = cur->nice;
      t->recent_cpu = cur->recent_cpu;
    }

  kf = alloc_frame (t, sizeof *kf);
  kf->eip = NULL;
  kf->function = function;
  kf->aux = aux;

  ef = alloc_frame (t, sizeof *ef);
  ef->eip = (void (*) (void)) kernel_thread;

  sf = alloc_frame (t, sizeof *sf);
  sf->eip = switch_entry;
  sf->ebp = 0;

  thread_unblock (t);

  /* Preempção na criação */
  if (t->priority > thread_current()->priority) {
    thread_yield();
  }

  return tid;
}

void
thread_block (void) 
{
  ASSERT (!intr_context ());
  ASSERT (intr_get_level () == INTR_OFF);

  thread_current ()->status = THREAD_BLOCKED;
  schedule ();
}

void
thread_unblock (struct thread *t) 
{
  enum intr_level old_level;

  ASSERT (is_thread (t));

  old_level = intr_disable ();
  ASSERT (t->status == THREAD_BLOCKED);
  
  /* Inserção por ordem de prioridade */
  list_insert_ordered (&ready_list, &t->elem, thread_compare_priority, NULL);

  t->status = THREAD_READY;
  intr_set_level (old_level);
}

const char *
thread_name (void) 
{
  return thread_current ()->name;
}

struct thread *
thread_current (void) 
{
  struct thread *t = running_thread ();
  
  ASSERT (is_thread (t));
  ASSERT (t->status == THREAD_RUNNING);

  return t;
}

tid_t
thread_tid (void) 
{
  return thread_current ()->tid;
}

void
thread_exit (void) 
{
  ASSERT (!intr_context ());

#ifdef USERPROG
  process_exit ();
#endif

  intr_disable ();
  list_remove (&thread_current()->allelem);
  thread_current ()->status = THREAD_DYING;
  schedule ();
  NOT_REACHED ();
}

void
thread_yield (void) 
{
  struct thread *cur = thread_current ();
  enum intr_level old_level;
  
  ASSERT (!intr_context ());

  old_level = intr_disable ();
   
  if (cur != idle_thread) 
    {
      /* Inserção por ordem de prioridade (Round Robin p/ mesmas prioridades) */
      list_insert_ordered (&ready_list, &cur->elem, thread_compare_priority, NULL);
    }
   
  cur->status = THREAD_READY;
  schedule ();
  intr_set_level (old_level);
}

// Comparador para Alarme (Menor tempo primeiro)
bool thread_compare(const struct list_elem *a, const struct list_elem *b, void *aux UNUSED) {
  struct thread *ta = list_entry(a, struct thread, elem);
  struct thread *tb = list_entry(b, struct thread, elem);
  return ta->wakeup_tick < tb->wakeup_tick;
}

// Comparador de Prioridade (Maior prioridade primeiro)
bool thread_compare_priority(const struct list_elem *a, const struct list_elem *b, void *aux UNUSED) {
  struct thread *ta = list_entry(a, struct thread, elem);
  struct thread *tb = list_entry(b, struct thread, elem);
  return ta->priority > tb->priority;
}

void
thread_sleep (int64_t ticks) {
  struct thread *atual = thread_current();
  enum intr_level old_level;

  if (ticks <= 0) return;

  old_level = intr_disable();

  if (atual != idle_thread) {
    atual->wakeup_tick = ticks;
    list_insert_ordered(&sleep_list, &atual->elem, thread_compare, NULL); 
    thread_block(); 
  }

  intr_set_level(old_level);
}

void
thread_wakeup (int64_t current_tick) {
  struct list_elem *e = list_begin(&sleep_list);

  while (e != list_end(&sleep_list)) {
    struct thread *t = list_entry (e, struct thread, elem);

    if (t->wakeup_tick <= current_tick) {
      e = list_remove(e);
      thread_unblock(t);

      /* Preempção se a thread que acordou for mais importante */
      if (t->priority > thread_current()->priority) {
        intr_yield_on_return();
      }
    }
    else break; 
  }
}

void
thread_foreach (thread_action_func *func, void *aux)
{
  struct list_elem *e;

  ASSERT (intr_get_level () == INTR_OFF);

  for (e = list_begin (&all_list); e != list_end (&all_list);
       e = list_next (e))
    {
      struct thread *t = list_entry (e, struct thread, allelem);
      func (t, aux);
    }
}

void
thread_set_priority (int new_priority) 
{
  if (thread_mlfqs)
    return;

  struct thread *cur = thread_current ();
  cur->priority = new_priority;

  /* Preempção se prioridade baixar */
  if (!list_empty(&ready_list)) {
    struct thread *highest = list_entry(list_front(&ready_list), struct thread, elem);
    if (cur->priority < highest->priority) {
      thread_yield();
    }
  }
}

int
thread_get_priority (void) 
{
  return thread_current ()->priority;
}

void
thread_set_nice (int nice) 
{
  struct thread *cur = thread_current ();
  cur->nice = nice;
  
  if (thread_mlfqs) {
    mlfqs_update_one_priority (cur, NULL); 
    
    /* Preempção se prioridade baixar pelo nice */
    if (!list_empty(&ready_list)) {
      struct thread *highest = list_entry(list_front(&ready_list), struct thread, elem);
      if (cur->priority < highest->priority) {
        thread_yield();
      }
    }
  }
}

int
thread_get_nice (void) 
{
  return thread_current ()->nice;
}

int
thread_get_load_avg (void) 
{
  return FLOAT_ROUND(FLOAT_MULT_MIX(load_avg, 100));
}

int
thread_get_recent_cpu (void) 
{
  return FLOAT_ROUND(FLOAT_MULT_MIX(thread_current()->recent_cpu, 100)); 
}

static void
idle (void *idle_started_ UNUSED) 
{
  struct semaphore *idle_started = idle_started_;
  idle_thread = thread_current ();
  sema_up (idle_started);

  for (;;) 
    {
      intr_disable ();
      thread_block ();
      asm volatile ("sti; hlt" : : : "memory");
    }
}

static void
kernel_thread (thread_func *function, void *aux) 
{
  ASSERT (function != NULL);

  intr_enable ();       
  function (aux);       
  thread_exit ();       
}

struct thread *
running_thread (void) 
{
  uint32_t *esp;
  asm ("mov %%esp, %0" : "=g" (esp));
  return pg_round_down (esp);
}

static bool
is_thread (struct thread *t)
{
  return t != NULL && t->magic == THREAD_MAGIC;
}

static void
init_thread (struct thread *t, const char *name, int priority)
{
  enum intr_level old_level;

  ASSERT (t != NULL);
  ASSERT (PRI_MIN <= priority && priority <= PRI_MAX);
  ASSERT (name != NULL);

  memset (t, 0, sizeof *t);
  t->status = THREAD_BLOCKED;
  strlcpy (t->name, name, sizeof t->name);
  t->stack = (uint8_t *) t + PGSIZE;
  t->priority = priority;
  t->magic = THREAD_MAGIC;
  t->wakeup_tick = 0; 
  t->nice = 0; 
  t->recent_cpu = FLOAT_CONST(0); 
  old_level = intr_disable ();
  list_push_back (&all_list, &t->allelem);
  intr_set_level (old_level);
}

static void *
alloc_frame (struct thread *t, size_t size) 
{
  ASSERT (is_thread (t));
  ASSERT (size % sizeof (uint32_t) == 0);

  t->stack -= size;
  return t->stack;
}

static struct thread *
next_thread_to_run (void) 
{
  if (list_empty (&ready_list))
    return idle_thread;
  else
    return list_entry (list_pop_front (&ready_list), struct thread, elem);
}

void
thread_schedule_tail (struct thread *prev)
{
  struct thread *cur = running_thread ();
  
  ASSERT (intr_get_level () == INTR_OFF);

  cur->status = THREAD_RUNNING;
  thread_ticks = 0;

#ifdef USERPROG
  process_activate ();
#endif

  if (prev != NULL && prev->status == THREAD_DYING && prev != initial_thread) 
    {
      ASSERT (prev != cur);
      palloc_free_page (prev);
    }
}

static void
schedule (void) 
{
  struct thread *cur = running_thread ();
  struct thread *next = next_thread_to_run ();
  struct thread *prev = NULL;

  ASSERT (intr_get_level () == INTR_OFF);
  ASSERT (cur->status != THREAD_RUNNING);
  ASSERT (is_thread (next));

  if (cur != next)
    prev = switch_threads (cur, next);
  thread_schedule_tail (prev);
}

static tid_t
allocate_tid (void) 
{
  static tid_t next_tid = 1;
  tid_t tid;

  lock_acquire (&tid_lock);
  tid = next_tid++;
  lock_release (&tid_lock);

  return tid;
}

static void
mlfqs_update_load_avg (void)
{
  int ready_threads = list_size(&ready_list);

  if (thread_current () != idle_thread) {
    ready_threads++;
  }

  int part1 = FLOAT_MULT(f_59_60, load_avg); 
  int part2 = FLOAT_MULT_MIX(f_1_60, ready_threads); 
  load_avg = FLOAT_ADD(part1, part2);
}

static void
mlfqs_update_one_recent_cpu (struct thread *t, void *aux UNUSED)
{
  int f_2_load_avg = FLOAT_MULT_MIX(load_avg, 2); 
  int f_2_load_avg_p1 = FLOAT_ADD_MIX(f_2_load_avg, 1); 
  
  int coeff = FLOAT_DIV(f_2_load_avg, f_2_load_avg_p1);
  int term1 = FLOAT_MULT(coeff, t->recent_cpu);   
  t->recent_cpu = FLOAT_ADD_MIX(term1, t->nice); 
}

static void
mlfqs_update_all_recent_cpu (void)
{
  thread_foreach(mlfqs_update_one_recent_cpu, NULL);
}

static void
mlfqs_update_one_priority (struct thread *t, void *aux UNUSED)
{
  if (t == idle_thread) {
    return;
  }

  int recent_cpu_div_4_fp = FLOAT_DIV_MIX(t->recent_cpu, 4);
  int term1 = FLOAT_INT_PART(recent_cpu_div_4_fp);
  int term2 = t->nice * 2;
  int new_priority = PRI_MAX - term1 - term2;

  if (new_priority > PRI_MAX) {
    new_priority = PRI_MAX;
  } else if (new_priority < PRI_MIN) {
    new_priority = PRI_MIN;
  }
  
  t->priority = new_priority;
}

static void
mlfqs_update_all_priority (void)
{
  thread_foreach(mlfqs_update_one_priority, NULL);
}

uint32_t thread_stack_ofs = offsetof (struct thread, stack);