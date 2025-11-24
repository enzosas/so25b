// so.c
// sistema operacional
// simulador de computador
// so25b

// ---------------------------------------------------------------------
// INCLUDES {{{1
// ---------------------------------------------------------------------

#include "so.h"
#include "dispositivos.h"
#include "err.h"
#include "irq.h"
#include "memoria.h"
#include "programa.h"

#include <stdlib.h>
#include <stdbool.h>


// ---------------------------------------------------------------------
// CONSTANTES E TIPOS {{{1
// ---------------------------------------------------------------------

// --- CONFIGURAÇÃO DO ESCALONADOR ---
#define ESCALONADOR_ROUND_ROBIN 1
#define ESCALONADOR_PRIORIDADE  2



// ESCALONADOR ATIVO ----- MUDE AQUI --------
#define ESCALONADOR_ATIVO ESCALONADOR_ROUND_ROBIN
// #define ESCALONADOR_ATIVO ESCALONADOR_PRIORIDADE



// intervalo entre interrupções do relogio
#define INTERVALO_INTERRUPCAO 10   // numero de instrucoes executadas entre duas interrupcoes de relogio

// tamanho da tabela de processos
#define MAX_PROCESSOS 10

// quantum do escalonador: quantidade de interrupcoes relogio que um processo recebe a partir da execucao (eh resetado em cada execucao)
#define QUANTUM 10

// os estados em que um processo pode se encontrar
typedef enum {
  PRONTO,
  EXECUTANDO,
  BLOQUEADO,
  TERMINADO
} processo_estado_t;

// o motivo pelo qual um processo pode estar bloqueado
typedef enum {
  BLOQUEIO_NENHUM,
  BLOQUEIO_LE,
  BLOQUEIO_ESCR,
  BLOQUEIO_ESPERA
} processo_bloqueio_t;

// a estrutura com as informações de um processo (Process Control Block)
typedef struct {
  int pid;                      // identificador do processo
  processo_estado_t estado;     // estado atual do processo
  processo_bloqueio_t tipo_bloqueio; // motivo pelo qual está bloqueado
  
  // contexto da CPU (registradores salvos)
  int regA;
  int regX;
  int regPC;
  int regERRO;

  // informações de E/S
  dispositivo_id_t disp_entrada;  // dispositivo de entrada do processo
  dispositivo_id_t disp_saida;    // dispositivo de saída do processo

  // para a chamada de sistema SO_ESPERA_PROC
  int pid_esperado;             // pid do processo que este esta esperando

  float prioridade;             // prioridade do processo
  int tempo_inicio_execucao;     // tempo de inicio de execucao do processo

  // ---------METRICAS---------
  int tempo_criacao;
  int tempo_termino;
  int num_preempcoes;
  //entrada nos estados
  int vezes_pronto;
  int vezes_bloqueado;
  int vezes_executando; 
  //cont de tempo nos estados
  long tempo_total_pronto;
  long tempo_total_bloqueado;
  long tempo_total_executando;
  int tempo_entrou_no_estado_atual;
  //tempo medio das respostas
  long soma_tempo_resposta;
  int n_respostas;
  int tempo_desbloqueio;
} processo_t;

struct so_t {
  cpu_t *cpu;
  mem_t *mem;
  es_t *es;
  console_t *console;
  bool erro_interno;

  int regA, regX, regPC, regERRO; // copia do estado da CPU

  processo_t tabela_processos[MAX_PROCESSOS];
  int processo_atual_idx;
  int proximo_pid;

  // fila de processos prontos (guarda os indices da tabela de processos)
  int fila_prontos[MAX_PROCESSOS];
  int inicio_fila;
  int fim_fila;
  int n_prontos;

  //metricas do sistema
  int num_processos_criados;
  long tempo_ocioso;
  int cont_interrupcoes[N_IRQ];
  int num_preempcoes_total;

  // controle de quantum
  int quantum_restante;
};



// funcao de tratamento de interrupcao (entrada no SO)
static int so_trata_interrupcao(void *argC, int reg_A);

// funções auxiliares
// carrega o programa contido no arquivo na memória do processador; retorna end. inicial
static int so_carrega_programa(so_t *self, char *nome_do_executavel);
// copia para str da memória do processador, até copiar um 0 (retorna true) ou tam bytes
static bool copia_str_da_mem(int tam, char str[tam], mem_t *mem, int ender);


// ---------------------------------------------------------------------
// CRIAÇÃO {{{1
// ---------------------------------------------------------------------

so_t *so_cria(cpu_t *cpu, mem_t *mem, es_t *es, console_t *console)
{
  so_t *self = malloc(sizeof(*self));
  if (self == NULL) return NULL;

  self->cpu = cpu;
  self->mem = mem;
  self->es = es;
  self->console = console;
  self->erro_interno = false;
  self->num_processos_criados = 0;
  self->tempo_ocioso = 0;
  self->num_preempcoes_total = 0;
  for (int i = 0; i < N_IRQ; i++) {
    self->cont_interrupcoes[i] = 0;
  }

  // inicializa a tabela de processos
  for (int i = 0; i < MAX_PROCESSOS; i++) {
    self->tabela_processos[i].estado = TERMINADO; // marcar todos como livres/terminados
    self->tabela_processos[i].pid = -1; // e deixar sem pid
  }
  self->processo_atual_idx = -1; // Nenhum processo executando inicialmente
  self->proximo_pid = 1;

  // Inicializa a fila de prontos
  self->inicio_fila = 0;
  self->fim_fila = 0;
  self->n_prontos = 0;

  // Inicializa o quantum
  self->quantum_restante = 0;

  // quando a CPU executar uma instrução CHAMAC, deve chamar a função
  //   so_trata_interrupcao, com primeiro argumento um ptr para o SO
  cpu_define_chamaC(self->cpu, so_trata_interrupcao, self);

  return self;
}

void so_destroi(so_t *self)
{
  cpu_define_chamaC(self->cpu, NULL, NULL);
  free(self);
}

#if ESCALONADOR_ATIVO == ESCALONADOR_ROUND_ROBIN //devido a ter dois escalonadores

// --- FUNÇÕES NOVAS PARA A FILA ---
// Insere um processo (pelo seu índice na tabela) no fim da fila de prontos
static void insere_fila_prontos(so_t *self, int processo_idx)
{
  if (self->n_prontos == MAX_PROCESSOS) {
    console_printf("SO: ERRO! Fila de prontos cheia.");
    return;
  }
  self->fila_prontos[self->fim_fila] = processo_idx;
  self->fim_fila = (self->fim_fila + 1) % MAX_PROCESSOS;
  self->n_prontos++;
}

// Remove e retorna o processo do início da fila de prontos
static int remove_fila_prontos(so_t *self)
{
  if (self->n_prontos == 0) {
    return -1; // Fila vazia
  }
  int processo_idx = self->fila_prontos[self->inicio_fila];
  self->inicio_fila = (self->inicio_fila + 1) % MAX_PROCESSOS;
  self->n_prontos--;
  return processo_idx;
}

#endif

//GERAR RELATORIO
void so_gera_relatorio(so_t *self)
{
    int tempo_final;
  es_le(self->es, D_RELOGIO_INSTRUCOES, &tempo_final);

  console_printf("\n\n--- RELATORIO FINAL DO SISTEMA ---");

  // --- Métricas Globais ---
  console_printf("\n[Metricas Globais]");
  console_printf("  - Tempo total de execucao: %d instrucoes", tempo_final);
  console_printf("  - Numero total de processos criados: %d", self->num_processos_criados);
  console_printf("  - Tempo em que a CPU ficou ociosa: %ld instrucoes", self->tempo_ocioso);
  console_printf("  - Numero total de preempcoes: %d", self->num_preempcoes_total);
  console_printf("  - Numero de interrupcoes por tipo:");
  for (int i = 0; i < N_IRQ; i++) {
    if (self->cont_interrupcoes[i] > 0) {
      console_printf("    > IRQ %d (%s): %d vezes", i, irq_nome(i), self->cont_interrupcoes[i]);
    }
  }

  int cont=0;
  // --- Métricas por Processo ---
  console_printf("\n[Metricas por Processo]");
  for (int i = 0; i < MAX_PROCESSOS; i++) {
    processo_t *p = &self->tabela_processos[i];
    if (p->tempo_criacao > 0) { // Imprime apenas para processos que existiram
      console_printf("\n  >> Processo P%d:", cont); // O PID ainda será válido neste ponto
      
      // Tempo de retorno
      if (p->tempo_termino > 0) {
        console_printf("     - Tempo de retorno: %d instrucoes", (p->tempo_termino - p->tempo_criacao));
      } else {
        console_printf("     - (Processo ainda ativo no final da execucao)");
      }
      
      // Preempções
      console_printf("     - Numero de preempcoes: %d", p->num_preempcoes);

      // Vezes em cada estado
      console_printf("     - Entradas em PRONTO: %d, BLOQUEADO: %d, EXECUTANDO: %d",
                   p->vezes_pronto, p->vezes_bloqueado, p->vezes_executando);
      
      // Tempo em cada estado
      console_printf("     - Tempo total em PRONTO: %ld, BLOQUEADO: %ld, EXECUTANDO: %ld",
                   p->tempo_total_pronto, p->tempo_total_bloqueado, p->tempo_total_executando);

      // Tempo médio de resposta
      if (p->n_respostas > 0) {
        double tempo_medio = (double)p->soma_tempo_resposta / p->n_respostas;
        console_printf("     - Tempo medio de resposta: %.2f instrucoes", tempo_medio);
      } else {
        console_printf("     - Tempo medio de resposta: N/A (nunca foi desbloqueado)");
      }
    }
    cont++;
  }
  console_printf("\n--- FIM DO RELATORIO ---");
}


// ---------------------------------------------------------------------
// TRATAMENTO DE INTERRUPÇÃO {{{1
// ---------------------------------------------------------------------

// funções auxiliares para o tratamento de interrupção
static void so_salva_estado_da_cpu(so_t *self);
static void so_trata_irq(so_t *self, int irq);
static void so_trata_pendencias(so_t *self);
static void so_escalona(so_t *self);
static int so_despacha(so_t *self);

// função a ser chamada pela CPU quando executa a instrução CHAMAC, no tratador de
//   interrupção em assembly
// essa é a única forma de entrada no SO depois da inicialização
// na inicialização do SO, a CPU foi programada para chamar esta função para executar
//   a instrução CHAMAC
// a instrução CHAMAC só deve ser executada pelo tratador de interrupção
//
// o primeiro argumento é um ponteiro para o SO, o segundo é a identificação
//   da interrupção
// o valor retornado por esta função é colocado no registrador A, e pode ser
//   testado pelo código que está após o CHAMAC. No tratador de interrupção em
//   assembly esse valor é usado para decidir se a CPU deve retornar da interrupção
//   (e executar o código de usuário) ou executar PARA e ficar suspensa até receber
//   outra interrupção
static int so_trata_interrupcao(void *argC, int reg_A)
{
  so_t *self = argC;
  irq_t irq = reg_A;

  self->cont_interrupcoes[irq]++;
  // esse print polui bastante, recomendo tirar quando estiver com mais confiança
  console_printf("SO: recebi IRQ %d (%s)", irq, irq_nome(irq));
  // salva o estado da cpu no descritor do processo que foi interrompido
  so_salva_estado_da_cpu(self);
  // faz o atendimento da interrupção
  so_trata_irq(self, irq);
  // faz o processamento independente da interrupção
  so_trata_pendencias(self);
  // escolhe o próximo processo a executar
  so_escalona(self);
  // recupera o estado do processo escolhido
  return so_despacha(self);
}

static void so_salva_estado_da_cpu(so_t *self)
{
  if (self->processo_atual_idx == -1) {
    return;
  }
  
  // obtém um ponteiro para o PCB do processo que estava executando
  processo_t *p = &self->tabela_processos[self->processo_atual_idx];

  int tempo_agora;
  if (es_le(self->es, D_RELOGIO_INSTRUCOES, &tempo_agora) == ERR_OK) {

      // Para as métricas de tempo
      p->tempo_total_executando += tempo_agora - p->tempo_entrou_no_estado_atual;
      p->tempo_entrou_no_estado_atual = tempo_agora;

      // Para o cálculo da prioridade (somente se estiver usando esse escalonador)
      #if ESCALONADOR_ATIVO == ESCALONADOR_PRIORIDADE
        int t_exec = tempo_agora - p->tempo_inicio_execucao;
        double t_quantum = (double)(QUANTUM * INTERVALO_INTERRUPCAO);
        if (t_quantum > 0) {
            p->prioridade = (p->prioridade + (t_exec / t_quantum)) / 2.0;
        }
      #endif
  }

  // lê o estado da CPU que foi salvo na memória pela interrupção
  int pc, a, erro, x;
  if (mem_le(self->mem, CPU_END_PC, &pc) != ERR_OK ||
      mem_le(self->mem, CPU_END_A, &a) != ERR_OK ||
      mem_le(self->mem, CPU_END_erro, &erro) != ERR_OK ||
      mem_le(self->mem, 59, &x) != ERR_OK) { // X salvo pelo trata_int.asm
    console_printf("SO: erro na leitura dos registradores ao salvar contexto.");
    self->erro_interno = true;
    return;
  }

  // salva os valores no PCB do processo
  p->regPC = pc;
  p->regA = a;
  p->regERRO = erro;
  p->regX = x;

  // se o processo estava executando, agora ele está pronto para voltar pra fila
  if (p->estado == EXECUTANDO) {
    p->estado = PRONTO;
    p->vezes_pronto++; //metricas
  }
}

static void so_trata_pendencias(so_t *self)
{
  // Percorre a tabela de processos para encontrar processos bloqueados
  for (int i = 0; i < MAX_PROCESSOS; i++) 
  {
    processo_t *p = &self->tabela_processos[i];

    if (p->estado == BLOQUEADO) 
    {
      // O processo está bloqueado, verifica o motivo
      
      if (p->tipo_bloqueio == BLOQUEIO_LE) 
      {
        // Bloqueado à espera de LEITURA. O dispositivo já está livre?
        dispositivo_id_t teclado = p->disp_entrada;
        dispositivo_id_t teclado_ok = teclado + TERM_TECLADO_OK - TERM_TECLADO;
        int estado_dev;
        es_le(self->es, teclado_ok, &estado_dev);

        if (estado_dev != 0) 
        {
          //metricas
          int tempo_agora;
          es_le(self->es, D_RELOGIO_INSTRUCOES, &tempo_agora);
          p->tempo_total_bloqueado += tempo_agora - p->tempo_entrou_no_estado_atual;
          p->vezes_pronto++;
          p->tempo_desbloqueio = tempo_agora;
          p->tempo_entrou_no_estado_atual = tempo_agora;

          // dispositivo pronto
          int dado;
          es_le(self->es, teclado, &dado);
          p->regA = dado; // Coloca o resultado no registador A
          p->estado = PRONTO; // Desbloqueia o processo
          #if ESCALONADOR_ATIVO == ESCALONADOR_ROUND_ROBIN
          insere_fila_prontos(self, i); // Adiciona na fila
          #endif
          p->tipo_bloqueio = BLOQUEIO_NENHUM;
          console_printf("SO: Processo %d desbloqueado apos leitura.", p->pid);
        }
      } 
      else if (p->tipo_bloqueio == BLOQUEIO_ESCR) 
      {
        // Bloqueado à espera de ESCRITA. O dispositivo já está livre?
        dispositivo_id_t tela = p->disp_saida;
        dispositivo_id_t tela_ok = tela + TERM_TELA_OK - TERM_TELA;
        int estado_dev;
        es_le(self->es, tela_ok, &estado_dev);

        if (estado_dev != 0) 
        {
          //metricas
          int tempo_agora;
          es_le(self->es, D_RELOGIO_INSTRUCOES, &tempo_agora);
          p->tempo_total_bloqueado += tempo_agora - p->tempo_entrou_no_estado_atual;
          p->vezes_pronto++;
          p->tempo_desbloqueio = tempo_agora;
          p->tempo_entrou_no_estado_atual = tempo_agora;

          // dispositivo pronto
          int dado = p->regX; // O dado a escrever ainda está em regX
          es_escreve(self->es, tela, dado);
          p->regA = 0; // Retorna 0 (sucesso)
          p->estado = PRONTO; // Desbloqueia o processo
          #if ESCALONADOR_ATIVO == ESCALONADOR_ROUND_ROBIN
          insere_fila_prontos(self, i); // Adiciona na fila]
          #endif
          p->tipo_bloqueio = BLOQUEIO_NENHUM;
          console_printf("SO: Processo %d desbloqueado apos escrita.", p->pid);
        }
      } 
      else if (p->tipo_bloqueio == BLOQUEIO_ESPERA) 
      {
        // Bloqueado à espera que outro processo morra
        int pid_esperado = p->pid_esperado;
        bool esperado_terminou = true; // assume que terminou ate que se prove o contrario!
        
        // procura o processo esperado na tabela
        for (int j = 0; j < MAX_PROCESSOS; j++) 
        {
          if (self->tabela_processos[j].pid == pid_esperado) 
          {
              // encontrou! portanto nao terminou
              esperado_terminou = false;
              break;
          }
        }
        if (esperado_terminou) 
        {
            //metricas
            int tempo_agora;
            es_le(self->es, D_RELOGIO_INSTRUCOES, &tempo_agora);
            p->tempo_total_bloqueado += tempo_agora - p->tempo_entrou_no_estado_atual;
            p->vezes_pronto++;
            p->tempo_desbloqueio = tempo_agora;
            p->tempo_entrou_no_estado_atual = tempo_agora;


            // processo nao existe mais. desbloqueia!
            p->estado = PRONTO;
            #if ESCALONADOR_ATIVO == ESCALONADOR_ROUND_ROBIN
            insere_fila_prontos(self, i); // Adiciona na fila
            #endif
            p->tipo_bloqueio = BLOQUEIO_NENHUM;
            p->pid_esperado = -1;
            p->regA = 0; // Sucesso na espera
            console_printf("SO: Processo %d desbloqueado (pendencias) pois %d terminou.", p->pid, pid_esperado);
        }
      }
    }
  }
}

static void so_escalona(so_t *self)
{
  // guarda quem estava executando antes de o escalonador rodar.
  int idx_anterior = self->processo_atual_idx;

  
  #if ESCALONADOR_ATIVO == ESCALONADOR_ROUND_ROBIN

  console_printf("SO: Escalonador Round-Robin em acao.");
  processo_t *p_atual = NULL;
  if (self->processo_atual_idx != -1) {
    p_atual = &self->tabela_processos[self->processo_atual_idx];
  }

  // se o processo ainda tem quantum em uma irq relogio, deve voltar para a cpu
  if (self->processo_atual_idx != -1 && p_atual->estado == PRONTO && self->quantum_restante > 0)  
  {
    console_printf("SO: Processo atual ainda tem quantum. Sem escalonamento.");
    console_printf("SO: Processo segue. PID = %d", self->tabela_processos[self->processo_atual_idx].pid);
    return;
  }

  // se o processo que estava a ser executado foi preemptido e ainda está PRONTO, ele deve voltar para o fim da fila.
  if (self->processo_atual_idx != -1 && p_atual->estado == PRONTO) 
  {
    #if ESCALONADOR_ATIVO == ESCALONADOR_ROUND_ROBIN
    insere_fila_prontos(self, self->processo_atual_idx);
    #endif
  }
  // O próximo a ser executado é o primeiro da fila de prontos
  self->processo_atual_idx = remove_fila_prontos(self);

  #elif ESCALONADOR_ATIVO == ESCALONADOR_PRIORIDADE
  console_printf("SO: Escalonador por Prioridade em acao.");
  
  
  int melhor_idx = -1;
  double menor_prio = 2.0; // Valor inicial > 1.0

  // Percorre toda a tabela de processos em busca do candidato ideal.
  for (int i = 0; i < MAX_PROCESSOS; i++) 
  {
    processo_t *p = &self->tabela_processos[i];
    if (p->estado == PRONTO) 
    {
      if (p->prioridade < menor_prio) 
      {
        menor_prio = p->prioridade;
        melhor_idx = i;
      }
    }
  }

  if (idx_anterior != -1 && // havia alguem executando
      melhor_idx != -1 &&  // tem alguem para executar
      idx_anterior != melhor_idx &&  // nao sao iguais
      self->tabela_processos[idx_anterior].estado == PRONTO) // processo anterior estava PRONTO
  {
      // preempcao por prioridade
      processo_t *p_preemptado = &self->tabela_processos[idx_anterior];
      p_preemptado->num_preempcoes++;
      self->num_preempcoes_total++;
      console_printf("SO: Preempcao por prioridade! PID %d tomou a vez do PID %d", self->tabela_processos[melhor_idx].pid, p_preemptado->pid);
  }

  // Define o processo escolhido como o próximo a ser executado.
  self->processo_atual_idx = melhor_idx;

  #else
  // Se um valor inválido for definido em ESCALONADOR_ATIVO, o compilador dará um erro.
  #error "Nenhum escalonador valido foi selecionado em ESCALONADOR_ATIVO!"
  #endif
  
  // Se mudou o processo OU se o quantum do anterior acabou
  if (self->processo_atual_idx != idx_anterior || self->quantum_restante <= 0) 
  {
    self->quantum_restante = QUANTUM;
  }
  if (self->processo_atual_idx != -1) {
    console_printf("SO: Processo escolhido. PID = %d", self->tabela_processos[self->processo_atual_idx].pid);
  } else {
    console_printf("SO: Nenhum processo pronto. CPU ociosa.");
  }
}

static int so_despacha(so_t *self)
{
  // se não há processo a executar, avisa a CPU para parar
  if (self->processo_atual_idx == -1) 
  {
    return 1; // Retorna 1 para o assembly, que fará a CPU parar (PARA)
  }

  // obtém um ponteiro para o PCB do processo que vai executar
  processo_t *p = &self->tabela_processos[self->processo_atual_idx];

  // escreve o estado do processo na memória, de onde a CPU irá restaurá-lo
  if (mem_escreve(self->mem, CPU_END_PC, p->regPC) != ERR_OK ||
      mem_escreve(self->mem, CPU_END_A, p->regA) != ERR_OK ||
      mem_escreve(self->mem, CPU_END_erro, p->regERRO) != ERR_OK ||
      mem_escreve(self->mem, 59, p->regX) != ERR_OK) 
  {
    console_printf("SO: erro na escrita dos registradores ao despachar processo.");
    self->erro_interno = true;
    return 1; // Para a CPU em caso de erro
  }
  
  // marca o processo como executando
  p->estado = EXECUTANDO;
 
  int tempo_agora;
  if (es_le(self->es, D_RELOGIO_INSTRUCOES, &tempo_agora) == ERR_OK) 
  {
      // Para o cálculo da prioridade
      p->tempo_inicio_execucao = tempo_agora;
      // Para as métricas de tempo
      p->tempo_total_pronto += tempo_agora - p->tempo_entrou_no_estado_atual;
      p->vezes_executando++;
      p->tempo_entrou_no_estado_atual = tempo_agora;

      if (p->tempo_desbloqueio > 0) { //tempo medio de resposta
          int tempo_resposta = tempo_agora - p->tempo_desbloqueio;
          p->soma_tempo_resposta += tempo_resposta;
          p->n_respostas++;
          p->tempo_desbloqueio = 0; // Zera para a próxima vez
      }
  }

  // se houver algum erro interno no SO, para a CPU
  if (self->erro_interno) 
  {
    return 1;
  }

  return 0; // Retorna 0 para o assembly, que fará a CPU retornar da interrupção (RETI)
}


// ---------------------------------------------------------------------
// TRATAMENTO DE UMA IRQ {{{1
// ---------------------------------------------------------------------

// funções auxiliares para tratar cada tipo de interrupção
static void so_trata_reset(so_t *self);
static void so_trata_irq_chamada_sistema(so_t *self);
static void so_trata_irq_err_cpu(so_t *self);
static void so_trata_irq_relogio(so_t *self);
static void so_trata_irq_desconhecida(so_t *self, int irq);

static void so_trata_irq(so_t *self, int irq)
{
  // verifica o tipo de interrupção que está acontecendo, e atende de acordo
  switch (irq) 
  {
    case IRQ_RESET:
      so_trata_reset(self);
      break;
    case IRQ_SISTEMA:
      so_trata_irq_chamada_sistema(self);
      break;
    case IRQ_ERR_CPU:
      so_trata_irq_err_cpu(self);
      break;
    case IRQ_RELOGIO:
      so_trata_irq_relogio(self);
      break;
    default:
      so_trata_irq_desconhecida(self, irq);
  }
}

// chamada uma única vez, quando a CPU inicializa
static void so_trata_reset(so_t *self)
{
  int ender = so_carrega_programa(self, "trata_int.maq");
  if (ender != CPU_END_TRATADOR) 
  {
    console_printf("SO: problema na carga do programa de tratamento de interrupção");
    self->erro_interno = true;
  }

  // programa o relógio para gerar uma interrupção após INTERVALO_INTERRUPCAO
  if (es_escreve(self->es, D_RELOGIO_TIMER, INTERVALO_INTERRUPCAO) != ERR_OK) 
  {
    console_printf("SO: problema na programação do timer");
    self->erro_interno = true;
  }

  // Cria o primeiro processo (init)
  // carrega o programa 'init.maq' na memória
  ender = so_carrega_programa(self, "init.maq");

  // so_carrega_programa agora retorna -1 em caso de erro
  if (ender < 0) 
  { 
    console_printf("SO: problema na carga do programa inicial");
    self->erro_interno = true;
    return;
  }

  // encontra um slot livre na tabela de processos (será o 0)
  int processo_idx = 0; // O primeiro processo vai para o primeiro slot
  
  // preenche a estrutura do processo (PCB)
  processo_t *p = &self->tabela_processos[processo_idx];
  p->pid = self->proximo_pid++;
  p->estado = PRONTO; // Esta pronto para executar, mas ainda não esta na CPU
  p->regPC = ender; // O contador de programa aponta para o inicio do init
  p->regA = 0;
  p->regX = 0;
  p->regERRO = ERR_OK;
  p->pid_esperado = -1; // Nao esta esperando por ninguem
  p->tipo_bloqueio = BLOQUEIO_NENHUM;

  // Atribui os dispositivos de E/S padrão (Terminal A)
  p->disp_entrada = D_TERM_A_TECLADO;
  p->disp_saida = D_TERM_A_TELA;

  #if ESCALONADOR_ATIVO == ESCALONADOR_ROUND_ROBIN
  // coloca o primeiro processo na fila de prontos
  insere_fila_prontos(self, processo_idx);
  #endif

  // Define o processo atual como -1 para que o escalonador o retire da fila
  self->processo_atual_idx = -1;
  
  console_printf("SO: processo 'init' criado com PID %d e inserido na fila.", p->pid);
}

// interrupção gerada quando a CPU identifica um erro
static void so_trata_irq_err_cpu(so_t *self)
{
  err_t err = self->regERRO;  //SDFKSMNKJSEMNFCESLFNMCSLEFNC,SKEFNCLSEKNCF,LESFN
  console_printf("SO: IRQ não tratada -- erro na CPU: %s", err_nome(err));
  self->erro_interno = true;
}

// interrupção gerada quando o timer expira
static void so_trata_irq_relogio(so_t *self)
{
  //   Rearma o relógio para a próxima interrupção
  err_t e1, e2;
  e1 = es_escreve(self->es, D_RELOGIO_INTERRUPCAO, 0); // desliga o sinalizador de interrupção
  e2 = es_escreve(self->es, D_RELOGIO_TIMER, INTERVALO_INTERRUPCAO);
  if (e1 != ERR_OK || e2 != ERR_OK) 
  {
    console_printf("SO: problema da reinicialização do timer");
    self->erro_interno = true;
  }
  
  //metricas
  if (self->processo_atual_idx == -1) 
  {
    self->tempo_ocioso += INTERVALO_INTERRUPCAO;
  }

  // Se não havia processo a ser executado, não há quantum a decrementar
  if (self->processo_atual_idx == -1) 
  {
    return;
  }

  // Decrementa o quantum restante
  self->quantum_restante--;
  console_printf("SO: Interrupcao do relogio, quantum restante = %d", self->quantum_restante);

  // Se o quantum acabou, força a preempção
  if (self->quantum_restante <= 0) 
  {
    processo_t *p = &self->tabela_processos[self->processo_atual_idx];
    p->num_preempcoes++; // metricas individual 
    self->num_preempcoes_total++; //metricas do sistema
    console_printf("SO: Quantum esgotado para o processo %d. Preempcao.", self->tabela_processos[self->processo_atual_idx].pid);
  }
}

// foi gerada uma interrupção para a qual o SO não está preparado
static void so_trata_irq_desconhecida(so_t *self, int irq)
{
  console_printf("SO: não sei tratar IRQ %d (%s)", irq, irq_nome(irq));
  self->erro_interno = true;
}


// ---------------------------------------------------------------------
// CHAMADAS DE SISTEMA {{{1
// ---------------------------------------------------------------------

// funções auxiliares para cada chamada de sistema
static void so_chamada_le(so_t *self);
static void so_chamada_escr(so_t *self);
static void so_chamada_cria_proc(so_t *self);
static void so_chamada_mata_proc(so_t *self);
static void so_chamada_espera_proc(so_t *self);

static void so_trata_irq_chamada_sistema(so_t *self)
{
  // a identificação da chamada está no registrador A
  // t2: com processos, o reg A deve estar no descritor do processo corrente
  processo_t *p = &self->tabela_processos[self->processo_atual_idx];
  int id_chamada = p->regA; 
  console_printf("SO: chamada de sistema %d", id_chamada);
  switch (id_chamada) {
    case SO_LE:
      so_chamada_le(self);
      break;
    case SO_ESCR:
      so_chamada_escr(self);
      break;
    case SO_CRIA_PROC:
      so_chamada_cria_proc(self);
      break;
    case SO_MATA_PROC:
      so_chamada_mata_proc(self);
      break;
    case SO_ESPERA_PROC:
      so_chamada_espera_proc(self);
      break;
    default:
      console_printf("SO: chamada de sistema desconhecida (%d)", id_chamada);
      self->erro_interno = true;
  }
}

// implementação da chamada se sistema SO_LE
// faz a leitura de um dado da entrada corrente do processo, coloca o dado no reg A
static void so_chamada_le(so_t *self)
{
  processo_t *p = &self->tabela_processos[self->processo_atual_idx];
  dispositivo_id_t teclado = p->disp_entrada;
  dispositivo_id_t teclado_ok = teclado + TERM_TECLADO_OK - TERM_TECLADO;
  
  // Verifica se o dispositivo de entrada está pronto
  int estado;
  if (es_le(self->es, teclado_ok, &estado) != ERR_OK) {
    console_printf("SO: problema no acesso ao estado do teclado do processo %d", p->pid);
    self->erro_interno = true;
    p->regA = -1; // Retorna erro
    return;
  }

  if (estado != 0) {
    // Dispositivo pronto, realiza a leitura imediatamente
    int dado;
    if (es_le(self->es, teclado, &dado) != ERR_OK) {
      console_printf("SO: problema no acesso ao teclado do processo %d", p->pid);
      self->erro_interno = true;
      p->regA = -1; // Retorna erro
      return;
    }
    p->regA = dado; // Coloca o dado lido no registador A do processo
  } else {
    // Dispositivo não está pronto, bloqueia o processo
    console_printf("SO: Processo %d bloqueado esperando por entrada.", p->pid);
    p->estado = BLOQUEADO;
    p->vezes_bloqueado++; //metricas
    p->tipo_bloqueio = BLOQUEIO_LE;
    // Força o escalonador a escolher outro processo
    self->processo_atual_idx = -1;
  }
}

// implementação da chamada se sistema SO_ESCR
// escreve o valor do reg X na saída corrente do processo
static void so_chamada_escr(so_t *self)
{
  processo_t *p = &self->tabela_processos[self->processo_atual_idx];
  dispositivo_id_t tela = p->disp_saida;
  dispositivo_id_t tela_ok = tela + TERM_TELA_OK - TERM_TELA;

  // Verifica se o dispositivo de saída está pronto
  int estado;
  if (es_le(self->es, tela_ok, &estado) != ERR_OK) 
  {
    console_printf("SO: problema no acesso ao estado da tela do processo %d", p->pid);
    self->erro_interno = true;
    p->regA = -1; // Retorna erro
    return;
  }
  
  if (estado != 0) 
  {
    // Dispositivo pronto, realiza a escrita imediatamente
    console_printf("SO: escrita imediata.");
    int dado = p->regX; // O dado a escrever está no registador X do processo
    if (es_escreve(self->es, tela, dado) != ERR_OK) 
    {
      console_printf("SO: problema no acesso à tela do processo %d", p->pid);
      self->erro_interno = true;
      p->regA = -1; // Retorna erro
      return;
    }
    p->regA = 0; // Retorna 0 (sucesso)
  } 
  else 
  {
    // Dispositivo não está pronto, bloqueia o processo
    console_printf("SO: Processo %d bloqueado esperando por saida.", p->pid);
    p->estado = BLOQUEADO;
    p->vezes_bloqueado++; //metricas
    p->tipo_bloqueio = BLOQUEIO_ESCR;
    // Força o escalonador a escolher outro processo
    self->processo_atual_idx = -1;
  }
}

// implementação da chamada se sistema SO_CRIA_PROC
// cria um processo
static void so_chamada_cria_proc(so_t *self)
{
  // o processo que está chamando a criação é o processo pai
  processo_t *pai = &self->tabela_processos[self->processo_atual_idx];

  // achar um slot livre na tabela de processos
  int novo_idx = -1;
  for (int i = 0; i < MAX_PROCESSOS; i++) 
  {
    if (self->tabela_processos[i].estado == TERMINADO) 
    {
      novo_idx = i;
      break;
    }
  }

  // se não houver slot livre, retorna erro
  if (novo_idx == -1) 
  {
    pai->regA = -1; // Retorno de erro: tabela de processos cheia
    console_printf("SO: Nao foi possivel criar processo, tabela cheia.");
    return;
  }

  // ler o nome do programa a ser executado da memória do processo pai
  int ender_nome = pai->regX; // O endereço do nome está no registrador X do pai
  char nome_prog[100];
  if (!copia_str_da_mem(100, nome_prog, self->mem, ender_nome)) 
  {
    pai->regA = -1; // Retorno de erro: nome do programa inválido
    console_printf("SO: Nao foi possivel ler o nome do programa para o novo processo.");
    return;
  }

  // carregar o programa na memória
  int ender_carga = so_carrega_programa(self, nome_prog);
  if (ender_carga < 0) 
  {
    pai->regA = -1; // Retorno de erro: falha ao carregar o programa
    console_printf("SO: Nao foi possivel carregar o programa '%s'.", nome_prog);
    return;
  }

  //incrementa a metrica
  self->num_processos_criados++;

  // preencher o PCB do novo processo
  processo_t *novo = &self->tabela_processos[novo_idx];
  novo->pid = self->proximo_pid++;
  novo->estado = PRONTO;
  /*
  #if ESCALONADOR_ATIVO == ESCALONADOR_ROUND_ROBIN
  insere_fila_prontos(self, novo_idx);
  #endif*/
  novo->tipo_bloqueio = BLOQUEIO_NENHUM;
  novo->regPC = ender_carga;
  novo->regA = 0;
  novo->regX = 0;
  novo->regERRO = ERR_OK;
  novo->pid_esperado = -1;
  novo->prioridade = 0.5;
  //metricas
  novo->num_preempcoes = 0;
  novo->vezes_pronto = 0;
  novo->vezes_bloqueado = 0;
  novo->vezes_executando = 0;
  novo->tempo_total_pronto = 0;
  novo->tempo_total_bloqueado = 0;
  novo->tempo_total_executando = 0;
  novo->soma_tempo_resposta = 0;
  novo->n_respostas = 0;
  novo->tempo_desbloqueio = 0;
  novo->tempo_termino = 0;

  int tempo_de_criacao;
  if (es_le(self->es, D_RELOGIO_INSTRUCOES, &tempo_de_criacao) == ERR_OK) 
  {
    novo->tempo_criacao = tempo_de_criacao;
    novo->tempo_entrou_no_estado_atual = tempo_de_criacao; // Importante para as métricas
  }

  // Atribui um terminal com base no PID (0=A, 1=B, 2=C, 3=D)
  // Assumindo 4 terminais no total
  int terminal_idx = (novo->pid - 1) % 4; 
  // O ID base do terminal (D_TERM_A, D_TERM_B, etc) é D_TERM_A + 4 * indice
  int term_base = D_TERM_A + terminal_idx * 4;

  // Atribui os dispositivos de E/S corretos para o terminal calculado
  novo->disp_entrada = term_base + TERM_TECLADO;
  novo->disp_saida = term_base + TERM_TELA;

  #if ESCALONADOR_ATIVO == ESCALONADOR_ROUND_ROBIN
  // Coloca o novo processo no fim da fila de prontos
  insere_fila_prontos(self, novo_idx);
  #endif

  // Retornar o PID do novo processo no registrador A do pai
  pai->regA = novo->pid;

  console_printf("SO: Processo '%s' criado com PID %d.", nome_prog, novo->pid);

}

// implementação da chamada se sistema SO_MATA_PROC
// mata o processo com pid X (ou o processo corrente se X é 0)
static void so_chamada_mata_proc(so_t *self)
{
  processo_t *chamador = &self->tabela_processos[self->processo_atual_idx];
  int pid_alvo = chamador->regX; // O PID a ser morto está no registrador X

  // Se o PID alvo for 0, o processo quer se matar
  if (pid_alvo == 0) 
  {
    pid_alvo = chamador->pid;
  }

  // 1. Encontrar o processo a ser morto na tabela
  int idx_alvo = -1;
  for (int i = 0; i < MAX_PROCESSOS; i++) 
  {
    if (self->tabela_processos[i].pid == pid_alvo && self->tabela_processos[i].estado != TERMINADO) 
    {
      idx_alvo = i;
      break;
    }
  }

  // Se não encontrou o processo, retorna erro
  if (idx_alvo == -1) 
  {
    chamador->regA = -1; // Retorno de erro
    console_printf("SO: Tentativa de matar processo com PID %d, mas ele nao existe.", pid_alvo);
    return;
  }

  // Mudar o estado do processo para TERMINADO
  processo_t *alvo = &self->tabela_processos[idx_alvo];

  //metricas
  int tempo_agora;
  es_le(self->es, D_RELOGIO_INSTRUCOES, &tempo_agora);
  alvo->tempo_termino = tempo_agora;

  int pid_morto = alvo->pid; // Guarda o PID antes de o invalidar
  alvo->estado = TERMINADO;
  alvo->pid = -1; // Libera o PID

  console_printf("SO: Processo com PID %d terminado.", pid_alvo);

  // desbloqueia processos que estavam à espera do processo que morreu
  for (int i = 0; i < MAX_PROCESSOS; i++) 
  {
    processo_t *p = &self->tabela_processos[i];
    if (p->estado == BLOQUEADO && p->tipo_bloqueio == BLOQUEIO_ESPERA && p->pid_esperado == pid_morto) 
    {
      
      p->estado = PRONTO;
      p->tipo_bloqueio = BLOQUEIO_NENHUM;
      p->pid_esperado = -1;
      p->regA = 0; // Retorna 0 (sucesso) para a chamada SO_ESPERA_PROC

      console_printf("SO: Processo %d desbloqueado pois processo %d terminou.", p->pid, pid_morto);
    }
  }

  if (alvo->pid == chamador->pid) 
  {
      self->processo_atual_idx = -1;
  }

  chamador->regA = 0;
}

// implementação da chamada se sistema SO_ESPERA_PROC
// espera o fim do processo com pid X
static void so_chamada_espera_proc(so_t *self)
{
  processo_t *chamador = &self->tabela_processos[self->processo_atual_idx];
  int pid_alvo = chamador->regX; // O PID a ser esperado está no registador X

  // validar o PID alvo
  // nao pode esperar por si mesmo
  if (pid_alvo == chamador->pid) 
  {
    chamador->regA = -1; // Retorna erro
    console_printf("SO: Processo %d tentou esperar por si mesmo.", chamador->pid);
    return;
  }

  // o processo alvo tem de existir e não pode estar ja terminado
  int idx_alvo = -1;
  for (int i = 0; i < MAX_PROCESSOS; i++) 
  {
    if (self->tabela_processos[i].pid == pid_alvo && self->tabela_processos[i].estado != TERMINADO) 
    {
      idx_alvo = i;
      break;
    }
  }

  // Se nao encontrou um processo valido para esperar, retorna erro
  if (idx_alvo == -1) 
  {
    chamador->regA = -1; // Retorna erro
    console_printf("SO: Processo %d tentou esperar por PID %d, que nao existe.", chamador->pid, pid_alvo);
    return;
  }

  // tudo válido!! bloqueia o processo chamador
  chamador->estado = BLOQUEADO;
  chamador->vezes_bloqueado++; //metricas
  chamador->tipo_bloqueio = BLOQUEIO_ESPERA;
  chamador->pid_esperado = pid_alvo;

  // Força o escalonador a escolher outro processo
  self->processo_atual_idx = -1;

  console_printf("SO: Processo %d bloqueado, esperando pelo processo %d.", chamador->pid, pid_alvo);
}


// ---------------------------------------------------------------------
// CARGA DE PROGRAMA {{{1
// ---------------------------------------------------------------------

// carrega o programa na memória
// retorna o endereço de carga ou -1
static int so_carrega_programa(so_t *self, char *nome_do_executavel)
{
  // programa para executar na nossa CPU
  programa_t *prog = prog_cria(nome_do_executavel);
  if (prog == NULL) 
  {
    console_printf("Erro na leitura do programa '%s'\n", nome_do_executavel);
    return -1;
  }

  int end_ini = prog_end_carga(prog);
  int end_fim = end_ini + prog_tamanho(prog);

  for (int end = end_ini; end < end_fim; end++) 
  {
    if (mem_escreve(self->mem, end, prog_dado(prog, end)) != ERR_OK) 
    {
      console_printf("Erro na carga da memória, endereco %d\n", end);
      return -1;
    }
  }

  prog_destroi(prog);
  console_printf("SO: carga de '%s' em %d-%d", nome_do_executavel, end_ini, end_fim);
  return end_ini;
}


// ---------------------------------------------------------------------
// ACESSO À MEMÓRIA DOS PROCESSOS {{{1
// ---------------------------------------------------------------------

// copia uma string da memória do simulador para o vetor str.
// retorna false se erro (string maior que vetor, valor não char na memória,
//   erro de acesso à memória)
// t2: deveria verificar se a memória pertence ao processo
static bool copia_str_da_mem(int tam, char str[tam], mem_t *mem, int ender)
{
  for (int indice_str = 0; indice_str < tam; indice_str++) {
    int caractere;
    if (mem_le(mem, ender + indice_str, &caractere) != ERR_OK) {
      return false;
    }
    if (caractere < 0 || caractere > 255) {
      return false;
    }
    str[indice_str] = caractere;
    if (caractere == 0) {
      return true;
    }
  }
  // estourou o tamanho de str
  return false;
}

// vim: foldmethod=marker
