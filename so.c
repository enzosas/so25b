// so.c
// sistema operacional
// simulador de computador
// so25b

// ---------------------------------------------------------------------
// INCLUDES {{{1
// ---------------------------------------------------------------------

#include "so.h"
#include "cpu.h"
#include "dispositivos.h"
#include "err.h"
#include "irq.h"
#include "memoria.h"
#include "programa.h"
#include "tabpag.h"
#include "mmu.h"

#include <stdlib.h>
#include <stdbool.h>


// ---------------------------------------------------------------------
// CONSTANTES E TIPOS {{{1
// ---------------------------------------------------------------------

// --- CONFIGURAÇÃO DO ESCALONADOR ---
#define ESCALONADOR_ROUND_ROBIN 1
#define ESCALONADOR_PRIORIDADE  2

// Para escolher qual escalonador usar, mude o valor abaixo e recompile.
#define ESCALONADOR_ATIVO ESCALONADOR_ROUND_ROBIN
//#define ESCALONADOR_ATIVO ESCALONADOR_PRIORIDADE

// intervalo entre interrupções do relógio
#define INTERVALO_INTERRUPCAO 50   // em instruções executadas

// a tabela de processos
#define MAX_PROCESSOS 10

// Quantum do escalonador: número de interrupções de relógio por processo
#define QUANTUM 2

#define NENHUM_PROCESSO -1
#define ALGUM_PROCESSO 0

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
  BLOQUEIO_ESPERA,
  BLOQUEIO_PAGINACAO
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
  int regComplemento;

  // informações de E/S
  dispositivo_id_t disp_entrada;  // dispositivo de entrada do processo
  dispositivo_id_t disp_saida;    // dispositivo de saída do processo

  // para a chamada de sistema SO_ESPERA_PROC
  int pid_esperado;             // pid do processo que este está esperando
  // PARTE 3 - T2
  float prioridade;             // Prioridade do processo
  int tempo_inicio_execucao;     // tempo de início de execução do processo

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

  // --- NOVO DO T3 ---
  // Cada processo agora tem a sua própria tabela de páginas
  tabpag_t *tabpag;

  // --- NOVOS DO T3 (Implementacao) ---
  char nome_executavel[100]; // Nome do arquivo para recarregar paginas
  int tam_memoria;           // Tamanho total (em bytes) da memoria virtual
  long tempo_termino_io_disco; // Tempo que a E/S de disco terminara
  int num_page_faults;       // Metrica: contagem de page faults

} processo_t;

struct so_t {
  cpu_t *cpu;
  mem_t *mem;
  mmu_t *mmu;
  es_t *es;
  console_t *console;
  bool erro_interno;

  // t2: tabela de processos, processo corrente, pendências, etc
  processo_t tabela_processos[MAX_PROCESSOS];
  int processo_atual_idx;
  int proximo_pid;

  // Fila de processos prontos (guarda os índices da tabela de processos)
  int fila_prontos[MAX_PROCESSOS];
  int inicio_fila;
  int fim_fila;
  int n_prontos;

  //métricas do sistema
  int num_processos_criados;
  long tempo_ocioso;
  int cont_interrupcoes[N_IRQ]; // N_IRQ é uma constante já definida
  int num_preempcoes_total;

  // Controle de quantum
  int quantum_restante;

  // T3
  // Gestão simples de memória física
  int quadro_livre;

  long tempo_disco_livre; // Tempo global em que o disco ficara livre
};

// --- DECLARAÇÕES ANTECIPADAS (PROTÓTIPOS) ---

// Função de tratamento de interrupção (entrada no SO)
static int so_trata_interrupcao(void *argC, int reg_A);

// Funções auxiliares gerais
static int so_carrega_programa(so_t *self, int processo_idx, char *nome_do_executavel);
static int so_carrega_programa_na_memoria_fisica(so_t *self, programa_t *programa);
static int so_carrega_programa_na_memoria_virtual(so_t *self, programa_t *programa, processo_t *processo, const char *nome_prog);
static bool so_copia_str_do_processo(so_t *self, int tam, char str[tam], int end_virt, int processo_idx);

// Funções do ciclo de tratamento de interrupção
static void so_salva_estado_da_cpu(so_t *self);
static void so_trata_irq(so_t *self, int irq);
static void so_trata_pendencias(so_t *self);
static void so_escalona(so_t *self);
static int so_despacha(so_t *self);

// Funções de tratamento para cada tipo de IRQ
static void so_trata_reset(so_t *self);
static void so_trata_irq_chamada_sistema(so_t *self);
static void so_trata_irq_err_cpu(so_t *self);
static void so_trata_irq_relogio(so_t *self);
static void so_trata_irq_desconhecida(so_t *self, int irq);

// Funções para cada chamada de sistema
static void so_chamada_le(so_t *self);
static void so_chamada_escr(so_t *self);
static void so_chamada_cria_proc(so_t *self);
static void so_chamada_mata_proc(so_t *self);
static void so_chamada_espera_proc(so_t *self);

// --- NOVOS PROTOTIPOS T3 ---
static void so_trata_falta_de_pagina(so_t *self);
static int so_encontra_quadro_livre(so_t *self);
static void so_carrega_pagina_do_disco(so_t *self, processo_t *p, int pagina_virtual, int quadro_destino);

#if ESCALONADOR_ATIVO == ESCALONADOR_ROUND_ROBIN
// Funções da fila (apenas se Round Robin estiver ativo)
static void insere_fila_prontos(so_t *self, int processo_idx);
static int remove_fila_prontos(so_t *self);
#endif

// Função de relatório
void so_gera_relatorio(so_t *self); // <-- Tornar esta não-estática se chamada de fora

// função de tratamento de interrupção (entrada no SO)
static int so_trata_interrupcao(void *argC, int reg_A);

// funções auxiliares
// carrega o programa contido no arquivo na memória do processador; retorna end. inicial
static int so_carrega_programa(so_t *self, int processo_idx, char *nome_do_executavel);

// ---------------------------------------------------------------------
// CRIAÇÃO {{{1
// ---------------------------------------------------------------------

so_t *so_cria(cpu_t *cpu, mem_t *mem, mmu_t *mmu, es_t *es, console_t *console)
{
  so_t *self = malloc(sizeof(*self));
  if (self == NULL) return NULL;

  self->cpu = cpu;
  self->mem = mem;
  self->mmu = mmu;
  self->es = es;
  self->console = console;
  self->erro_interno = false;
  self->num_processos_criados = 0;
  self->tempo_ocioso = 0;
  self->num_preempcoes_total = 0;
  for (int i = 0; i < N_IRQ; i++) {
    self->cont_interrupcoes[i] = 0;
  }

  // Inicializa a tabela de processos
  for (int i = 0; i < MAX_PROCESSOS; i++) {
    self->tabela_processos[i].estado = TERMINADO; // Marcar todos como livres/terminados
    self->tabela_processos[i].pid = -1;
  }
  self->processo_atual_idx = NENHUM_PROCESSO; // Nenhum processo executando inicialmente
  self->proximo_pid = 1;

  // Inicializa a fila de prontos
  self->inicio_fila = 0;
  self->fim_fila = 0;
  self->n_prontos = 0;

  // Inicializa o quantum
  self->quantum_restante = 0;
  
  // --- T3 ---
  // Inicializa o gestor de memória (simplificado)
  // O primeiro quadro livre é após a memória protegida pelo hardware
  self->quadro_livre = CPU_END_FIM_PROT / TAM_PAGINA + 1;

  self->tempo_disco_livre = 0; // Disco comeca livre

  // quando a CPU executar uma instrução CHAMAC, deve chamar a função
  //   so_trata_interrupcao, com primeiro argumento um ptr para o SO
  cpu_define_chamaC(self->cpu, so_trata_interrupcao, self);

  // --- T3 ---
  // Desliga a MMU no início (sem tabela de páginas global)
  mmu_define_tabpag(self->mmu, NULL);

  return self;
}

void so_destroi(so_t *self)
{
  cpu_define_chamaC(self->cpu, NULL, NULL);

  // --- T3 ---
  // Limpa as tabelas de páginas de processos que possam ter sobrado
  for (int i = 0; i < MAX_PROCESSOS; i++) {
    if (self->tabela_processos[i].tabpag != NULL) {
      tabpag_destroi(self->tabela_processos[i].tabpag);
    }
  }

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
  // t2: salva os registradores que compõem o estado da cpu no descritor do
  //   processo corrente. os valores dos registradores foram colocados pela
  //   CPU na memória, nos endereços CPU_END_PC etc. O registrador X foi salvo
  //   pelo tratador de interrupção (ver trata_irq.asm) no endereço 59
  // se não houver processo corrente, não faz nada
  if (self->processo_atual_idx == NENHUM_PROCESSO) {
    return;
  }
  
  // obtém um ponteiro para o PCB do processo que estava executando
  processo_t *p = &self->tabela_processos[self->processo_atual_idx];

  // PARTE 3 - T2
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
  int comp; // T3

  if (mem_le(self->mem, CPU_END_PC, &pc) != ERR_OK ||
      mem_le(self->mem, CPU_END_A, &a) != ERR_OK ||
      mem_le(self->mem, CPU_END_erro, &erro) != ERR_OK ||
      mem_le(self->mem, 59, &x) != ERR_OK || // X salvo pelo trata_int.asm
      mem_le(self->mem, CPU_END_complemento, &comp) != ERR_OK) { 
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
  // t2: realiza ações que não são diretamente ligadas com a interrupção que
  //   está sendo atendida:
  // - E/S pendente
  // - desbloqueio de processos
  // - contabilidades
  // - etc

  // Percorre a tabela de processos para encontrar processos bloqueados
  for (int i = 0; i < MAX_PROCESSOS; i++) {
    processo_t *p = &self->tabela_processos[i];

    if (p->estado == BLOQUEADO) {
      // O processo está bloqueado, verifica o motivo
      
      if (p->tipo_bloqueio == BLOQUEIO_LE) {
        // Bloqueado à espera de LEITURA. O dispositivo já está livre?
        dispositivo_id_t teclado = p->disp_entrada;
        dispositivo_id_t teclado_ok = teclado + TERM_TECLADO_OK - TERM_TECLADO;
        int estado_dev;
        es_le(self->es, teclado_ok, &estado_dev);

        if (estado_dev != 0) {
          //metricas
          int tempo_agora;
          es_le(self->es, D_RELOGIO_INSTRUCOES, &tempo_agora);
          p->tempo_total_bloqueado += tempo_agora - p->tempo_entrou_no_estado_atual;
          p->vezes_pronto++;
          p->tempo_desbloqueio = tempo_agora;
          p->tempo_entrou_no_estado_atual = tempo_agora;


          // Sim, o dispositivo está pronto! Completa a operação de leitura.
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

      } else if (p->tipo_bloqueio == BLOQUEIO_ESCR) {
        // Bloqueado à espera de ESCRITA. O dispositivo já está livre?
        dispositivo_id_t tela = p->disp_saida;
        dispositivo_id_t tela_ok = tela + TERM_TELA_OK - TERM_TELA;
        int estado_dev;
        es_le(self->es, tela_ok, &estado_dev);

        if (estado_dev != 0) {
          //metricas
          int tempo_agora;
          es_le(self->es, D_RELOGIO_INSTRUCOES, &tempo_agora);
          p->tempo_total_bloqueado += tempo_agora - p->tempo_entrou_no_estado_atual;
          p->vezes_pronto++;
          p->tempo_desbloqueio = tempo_agora;
          p->tempo_entrou_no_estado_atual = tempo_agora;

          // Sim, o dispositivo está pronto! Completa a operação de escrita.
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

      } else if (p->tipo_bloqueio == BLOQUEIO_ESPERA) {
        // Bloqueado à espera que outro processo morra. Já morreu?
        int pid_esperado = p->pid_esperado;
        bool esperado_terminou = true; // Assume que terminou, a não ser que o encontremos ativo
        
        // Procura o processo esperado na tabela
        for (int j = 0; j < MAX_PROCESSOS; j++) {
            if (self->tabela_processos[j].pid == pid_esperado) {
                // Encontrou, então não terminou
                esperado_terminou = false;
                break;
            }
        }

        if (esperado_terminou) {
            //metricas
            int tempo_agora;
            es_le(self->es, D_RELOGIO_INSTRUCOES, &tempo_agora);
            p->tempo_total_bloqueado += tempo_agora - p->tempo_entrou_no_estado_atual;
            p->vezes_pronto++;
            p->tempo_desbloqueio = tempo_agora;
            p->tempo_entrou_no_estado_atual = tempo_agora;


            // Sim, o processo esperado já não existe. Desbloqueia.
            p->estado = PRONTO;
            #if ESCALONADOR_ATIVO == ESCALONADOR_ROUND_ROBIN
            insere_fila_prontos(self, i); // Adiciona na fila
            #endif
            p->tipo_bloqueio = BLOQUEIO_NENHUM;
            p->pid_esperado = -1;
            p->regA = 0; // Sucesso na espera
            console_printf("SO: Processo %d desbloqueado (pendencias) pois %d terminou.", p->pid, pid_esperado);
        }
      } else if (p->tipo_bloqueio == BLOQUEIO_PAGINACAO) {
        // --- NOVO T3 ---
        // Bloqueado a espera de E/S de disco (Page Fault)
        int tempo_agora;
        es_le(self->es, D_RELOGIO_INSTRUCOES, &tempo_agora);
        long tempo_termino_io = p->tempo_termino_io_disco;

        if (tempo_agora >= tempo_termino_io) {
          // E/S de disco terminada! Desbloqueia o processo.
          console_printf("SO: Processo %d desbloqueado apos E/S de disco (Page Fault).", p->pid);
          
          //metricas
          p->tempo_total_bloqueado += tempo_agora - p->tempo_entrou_no_estado_atual;
          p->vezes_pronto++;
          p->tempo_desbloqueio = tempo_agora; // Correto para a metrica de tempo de resposta
          p->tempo_entrou_no_estado_atual = tempo_agora;

          // Desbloqueia
          p->estado = PRONTO;
          #if ESCALONADOR_ATIVO == ESCALONADOR_ROUND_ROBIN
          insere_fila_prontos(self, i); // Adiciona na fila
          #endif
          p->tipo_bloqueio = BLOQUEIO_NENHUM;
          p->tempo_termino_io_disco = 0;

          // IMPORTANTE: O processo foi interrompido *antes* de executar
          // a instrucao que causou a falha. O PC salvo aponta
          // para essa instrucao. Ao retornar, a CPU ira executa-la
          // novamente, mas agora a pagina esta mapeada.
        }
      }
    }
  }
}

static void so_escalona(so_t *self)
{
  // escolhe o próximo processo a executar, que passa a ser o processo
  //   corrente; pode continuar sendo o mesmo de antes ou não
  // t2: na primeira versão, escolhe um processo pronto caso o processo
  //   corrente não possa continuar executando, senão deixa o mesmo processo.
  //   depois, implementa um escalonador melhor
  // A diretiva de pré-processador #if verifica o valor de ESCALONADOR_ATIVO
  // antes mesmo de o programa ser compilado. Apenas o código do bloco
  // correspondente será incluído no executável final.

#if ESCALONADOR_ATIVO == ESCALONADOR_ROUND_ROBIN
  console_printf("SO: Escalonador Round-Robin em acao.");

  // Se o processo que estava a ser executado foi preemptido e ainda está PRONTO,
  // ele deve voltar para o fim da fila.
  if (self->processo_atual_idx != -1 &&
      self->tabela_processos[self->processo_atual_idx].estado == PRONTO) {
      #if ESCALONADOR_ATIVO == ESCALONADOR_ROUND_ROBIN
      insere_fila_prontos(self, self->processo_atual_idx);
      #endif
  }

  // O próximo a ser executado é o primeiro da fila de prontos
  self->processo_atual_idx = remove_fila_prontos(self);

#elif ESCALONADOR_ATIVO == ESCALONADOR_PRIORIDADE

  console_printf("SO: Escalonador por Prioridade em acao.");

   // 1. Guarda quem estava executando ANTES de o escalonador rodar.
  int idx_anterior = self->processo_atual_idx;//para contar preempcoes
  
  int melhor_idx = -1;
  double menor_prio = 2.0; // Valor inicial > 1.0

  // Percorre toda a tabela de processos em busca do candidato ideal.
  for (int i = 0; i < MAX_PROCESSOS; i++) {
    processo_t *p = &self->tabela_processos[i];
    if (p->estado == PRONTO) {
      if (p->prioridade < menor_prio) {
        menor_prio = p->prioridade;
        melhor_idx = i;
      }
    }
  }

  if (idx_anterior != -1 &&                      // Havia alguém executando
      melhor_idx != -1 &&                        // Encontramos um próximo para executar
      idx_anterior != melhor_idx &&               // E o escalonador escolheu um processo DIFERENTE
      self->tabela_processos[idx_anterior].estado == PRONTO) // E o processo anterior ainda estava PRONTO para rodar
  {
      // Isto é uma preempção por prioridade!
      processo_t *p_preemptado = &self->tabela_processos[idx_anterior];
      p_preemptado->num_preempcoes++;
      self->num_preempcoes_total++;
      
      console_printf("SO: Preempcao por prioridade! PID %d tomou a vez do PID %d",
                   self->tabela_processos[melhor_idx].pid,
                   p_preemptado->pid);
  } //para contar preempcoes por prioridade

  // Define o processo escolhido como o próximo a ser executado.
  self->processo_atual_idx = melhor_idx;

#else
  // Se um valor inválido for definido em ESCALONADOR_ATIVO, o compilador dará um erro.
  #error "Nenhum escalonador valido foi selecionado em ESCALONADOR_ATIVO!"
#endif
}

static int so_despacha(so_t *self)
{
  // t2: se houver processo corrente, coloca o estado desse processo onde ele
  //   será recuperado pela CPU (em CPU_END_PC etc e 59) e retorna 0,
  //   senão retorna 1
  // o valor retornado será o valor de retorno de CHAMAC, e será colocado no 
  //   registrador A para o tratador de interrupção (ver trata_irq.asm).
  
  // se não há processo a executar, avisa a CPU para parar
  if (self->processo_atual_idx == NENHUM_PROCESSO) {
    // --- ADICIONADO T3 ---
    // Diz à MMU para não usar nenhuma tabela (desliga a tradução)
    mmu_define_tabpag(self->mmu, NULL);
    return 1; // Retorna 1 para o assembly, que fará a CPU parar (PARA)
  }

  // obtém um ponteiro para o PCB do processo que vai executar
  processo_t *p = &self->tabela_processos[self->processo_atual_idx];

  // --- ADICIONADO T3 ---
  // Diz à MMU para usar a tabela de páginas deste processo
  mmu_define_tabpag(self->mmu, p->tabpag);

  self->quantum_restante = QUANTUM;

  // escreve o estado do processo na memória, de onde a CPU irá restaurá-lo
  if (mem_escreve(self->mem, CPU_END_PC, p->regPC) != ERR_OK ||
      mem_escreve(self->mem, CPU_END_A, p->regA) != ERR_OK ||
      mem_escreve(self->mem, CPU_END_erro, p->regERRO) != ERR_OK ||
      mem_escreve(self->mem, 59, p->regX) != ERR_OK) {
    console_printf("SO: erro na escrita dos registradores ao despachar processo.");
    self->erro_interno = true;
    return 1; // Para a CPU em caso de erro
  }
  
  // marca o processo como executando
  p->estado = EXECUTANDO;
  // PARTE 3 - T2
  int tempo_agora;
  if (es_le(self->es, D_RELOGIO_INSTRUCOES, &tempo_agora) == ERR_OK) {
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
  if (self->erro_interno) {
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
  switch (irq) {
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
  // coloca o tratador de interrupção na memória
  // quando a CPU aceita uma interrupção, passa para modo supervisor,
  //   salva seu estado à partir do endereço CPU_END_PC, e desvia para o
  //   endereço CPU_END_TRATADOR
  // colocamos no endereço CPU_END_TRATADOR o programa de tratamento
  //   de interrupção (escrito em asm). esse programa deve conter a
  //   instrução CHAMAC, que vai chamar so_trata_interrupcao (como
  //   foi definido na inicialização do SO)
  
  // t2: deveria criar um processo para o init, e inicializar o estado do
  //   processador para esse processo com os registradores zerados, exceto
  //   o PC e o modo.
  // como não tem suporte a processos, está carregando os valores dos
  //   registradores diretamente no estado da CPU mantido pelo SO; daí vai
  //   copiar para o início da memória pelo despachante, de onde a CPU vai
  //   carregar para os seus registradores quando executar a instrução RETI
  //   em bios.asm (que é onde está a instrução CHAMAC que causou a execução
  //   deste código
  
  int ender = so_carrega_programa(self, NENHUM_PROCESSO, "trata_int.maq");
  if (ender != CPU_END_TRATADOR) {
    console_printf("SO: problema na carga do programa de tratamento de interrupção");
    self->erro_interno = true;
  }

  // programa o relógio para gerar uma interrupção após INTERVALO_INTERRUPCAO
  if (es_escreve(self->es, D_RELOGIO_TIMER, INTERVALO_INTERRUPCAO) != ERR_OK) {
    console_printf("SO: problema na programação do timer");
    self->erro_interno = true;
  }

  // Cria o primeiro processo (init)
  int processo_idx = 0; // O primeiro processo vai para o primeiro slot
  processo_t *p = &self->tabela_processos[processo_idx];

  // Cria a tabela de páginas para este processo ANTES de carregar
  p->tabpag = tabpag_cria();
  if (p->tabpag == NULL) {
      console_printf("SO: Falha ao criar tabela de paginas para init!");
      self->erro_interno = true;
      return;
  }
  
  // Carrega o programa 'init.maq' na memória VIRTUAL do processo
  ender = so_carrega_programa(self, processo_idx, "init.maq");
  if (ender < 0) { // so_carrega_programa agora retorna -1 em caso de erro
    console_printf("SO: problema na carga do programa inicial");
    self->erro_interno = true;
    return;
  }
  
  // Preenche a estrutura do processo (PCB)
  p->pid = self->proximo_pid++;
  p->estado = PRONTO; // Está pronto para executar, mas ainda não está na CPU
  p->regPC = ender;   // O contador de programa aponta para o início do init
  p->regA = 0;
  p->regX = 0;
  p->regERRO = ERR_OK;
  p->regComplemento = 0; // (T3) Inicializa o novo registador
  p->pid_esperado = -1; // Não está esperando por ninguém

  // --- NOVO T3 ---
  p->tam_memoria = 0; // Sera definido por so_carrega_programa
  p->nome_executavel[0] = '\0'; // Sera definido por so_carrega_programa
  p->tempo_termino_io_disco = 0;
  p->num_page_faults = 0; // O init nao deve ter page faults se for carregado direto

  p->tipo_bloqueio = BLOQUEIO_NENHUM;

  // Atribui os dispositivos de E/S padrão (Terminal A)
  p->disp_entrada = D_TERM_A_TECLADO;
  p->disp_saida = D_TERM_A_TELA;

  #if ESCALONADOR_ATIVO == ESCALONADOR_ROUND_ROBIN
  // 4. Coloca o primeiro processo na fila de prontos
  insere_fila_prontos(self, processo_idx);
  #endif

  // Define o processo atual como -1 para que o escalonador o retire da fila
  self->processo_atual_idx = -1;

  // Não alteramos mais self->regPC diretamente. O despachante usará o 
  // valor do processo atual.
  
  console_printf("SO: processo 'init' criado com PID %d e inserido na fila.", p->pid);
}

// interrupção gerada quando a CPU identifica um erro
static void so_trata_irq_err_cpu(so_t *self)
{
  // Ocorreu um erro interno na CPU
  // O erro está codificado em CPU_END_erro
  // Em geral, causa a morte do processo que causou o erro
  // Ainda não temos processos, causa a parada da CPU
  // t2: com suporte a processos, deveria pegar o valor do registrador erro
  //   no descritor do processo corrente, e reagir de acordo com esse erro
  //   (em geral, matando o processo)
  
  // Se não havia processo (NENHUM_PROCESSO), é um erro grave do SO.
  if (self->processo_atual_idx == NENHUM_PROCESSO) {
    console_printf("SO: ERRO FATAL DE CPU SEM PROCESSO ATIVO!");
    self->erro_interno = true;
    return;
  }

  // Pega o erro do PCB do processo que o causou
  processo_t *p = &self->tabela_processos[self->processo_atual_idx];
  err_t err = p->regERRO;
  int complemento = p->regComplemento; // (T3) Pega a info extra
  
  // (Lógica T3) Imprime uma mensagem de erro detalhada
  console_printf("SO: Processo PID %d causou erro de CPU: %s", p->pid, err_nome(err));
  if (err == ERR_PAG_AUSENTE) 
  {
    console_printf("SO:   -> PAGE FAULT no endereco virtual %d", complemento);
    // Chama o tratador especifico
    so_trata_falta_de_pagina(self);
    return; // O tratador decide se mata ou bloqueia o processo

  } 
  else if (err == ERR_END_INV) 
  {
    console_printf("SO:   -> ENDERECO INVALIDO (fisico) %d. Erro grave do SO.", complemento);
  } 
  else 
  {
    console_printf("SO:   -> Info adicional (complemento): %d", complemento);
  }

  // Mata o processo que causou o erro.
  // A forma mais limpa de fazer isso é forçar uma chamada de "suicídio"
  // e deixar a função so_chamada_mata_proc fazer a limpeza.
  console_printf("SO: Matando processo %d devido ao erro.", p->pid);
  p->regX = 0; // 0 significa "matar a si mesmo"
  so_chamada_mata_proc(self);

}

// interrupção gerada quando o timer expira
static void so_trata_irq_relogio(so_t *self)
{
  // t2: deveria tratar a interrupção
  //   por exemplo, decrementa o quantum do processo corrente, quando se tem
  //   um escalonador com quantum
  
  //   Rearma o relógio para a próxima interrupção
  err_t e1, e2;
  e1 = es_escreve(self->es, D_RELOGIO_INTERRUPCAO, 0); // desliga o sinalizador de interrupção
  e2 = es_escreve(self->es, D_RELOGIO_TIMER, INTERVALO_INTERRUPCAO);
  if (e1 != ERR_OK || e2 != ERR_OK) {
    console_printf("SO: problema da reinicialização do timer");
    self->erro_interno = true;
  }
  
  //metricas
  if (self->processo_atual_idx == -1) {
    self->tempo_ocioso += INTERVALO_INTERRUPCAO;
  }

  // Se não havia processo a ser executado, não há quantum a decrementar
  if (self->processo_atual_idx == -1) {
    return;
  }

  // 2. Decrementa o quantum restante
  self->quantum_restante--;
  
  console_printf("SO: Interrupcao do relogio, quantum restante = %d", self->quantum_restante);

  // 3. Se o quantum acabou, força a preempção
  if (self->quantum_restante <= 0) {
    //parte 3 t2
    processo_t *p = &self->tabela_processos[self->processo_atual_idx];
    p->num_preempcoes++; // metricas individual 
    self->num_preempcoes_total++; //metricas do sistema

    console_printf("SO: Quantum esgotado para o processo %d. Preempcao.",
                   self->tabela_processos[self->processo_atual_idx].pid);
    // O escalonador irá tratar de colocar o processo atual no fim da fila
    // e escolher o próximo.
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
      // t2: deveria matar o processo
      self->erro_interno = true;
  }
}

// implementação da chamada se sistema SO_LE
// faz a leitura de um dado da entrada corrente do processo, coloca o dado no reg A
static void so_chamada_le(so_t *self)
{
  // implementação com espera ocupada
  //   t2: deveria realizar a leitura somente se a entrada estiver disponível,
  //     senão, deveria bloquear o processo.
  //   no caso de bloqueio do processo, a leitura (e desbloqueio) deverá
  //     ser feita mais tarde, em tratamentos pendentes em outra interrupção,
  //     ou diretamente em uma interrupção específica do dispositivo, se for
  //     o caso
  // implementação lendo direto do terminal A
  //   t2: deveria usar dispositivo de entrada corrente do processo
  
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
  // implementação com espera ocupada
  //   t2: deveria bloquear o processo se dispositivo ocupado
  // implementação escrevendo direto do terminal A
  //   t2: deveria usar o dispositivo de saída corrente do processo

  processo_t *p = &self->tabela_processos[self->processo_atual_idx];
  dispositivo_id_t tela = p->disp_saida;
  dispositivo_id_t tela_ok = tela + TERM_TELA_OK - TERM_TELA;

  // Verifica se o dispositivo de saída está pronto
  int estado;
  if (es_le(self->es, tela_ok, &estado) != ERR_OK) {
    console_printf("SO: problema no acesso ao estado da tela do processo %d", p->pid);
    self->erro_interno = true;
    p->regA = -1; // Retorna erro
    return;
  }
  
  if (estado != 0) {
    // Dispositivo pronto, realiza a escrita imediatamente
    int dado = p->regX; // O dado a escrever está no registador X do processo
    if (es_escreve(self->es, tela, dado) != ERR_OK) {
      console_printf("SO: problema no acesso à tela do processo %d", p->pid);
      self->erro_interno = true;
      p->regA = -1; // Retorna erro
      return;
    }
    p->regA = 0; // Retorna 0 (sucesso)
  } else {
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

  // 1. Achar um slot livre na tabela de processos
  int novo_idx = -1;
  for (int i = 0; i < MAX_PROCESSOS; i++) {
    if (self->tabela_processos[i].estado == TERMINADO) {
      novo_idx = i;
      break;
    }
  }

  // Se não houver slot livre, retorna erro
  if (novo_idx == -1) {
    pai->regA = -1; // Retorno de erro: tabela de processos cheia
    console_printf("SO: Nao foi possivel criar processo, tabela cheia.");
    return;
  }

  // T3
  // 2. Ler o nome do programa a ser executado da memória VIRTUAL do processo pai
  int ender_nome = pai->regX; // O endereço do nome está no registrador X do pai
  char nome_prog[100];
  if (!so_copia_str_do_processo(self, 100, nome_prog, ender_nome, self->processo_atual_idx)) {
    pai->regA = -1; // Retorno de erro: nome do programa inválido
    console_printf("SO: Nao foi possivel ler o nome do programa para o novo processo.");
    return;
  }

  //incrementa a metrica
  self->num_processos_criados++;

  // 3. Preencher o PCB do novo processo
  processo_t *novo = &self->tabela_processos[novo_idx];

  // --- T3 ---
  // 3a. Cria a tabela de páginas para este processo ANTES de carregar
  novo->tabpag = tabpag_cria();
  if (novo->tabpag == NULL) {
      console_printf("SO: Falha ao criar tabela de paginas para PID %d!", self->proximo_pid);
      pai->regA = -1;
      return;
  }
  // 3b. Carregar o programa na memória VIRTUAL do novo processo
  int ender_carga = so_carrega_programa(self, novo_idx, nome_prog);
  if (ender_carga < 0) {
    pai->regA = -1; // Retorno de erro: falha ao carregar o programa
    console_printf("SO: Nao foi possivel carregar o programa '%s'.", nome_prog);
    tabpag_destroi(novo->tabpag); // Limpa a tabela de páginas criada
    novo->tabpag = NULL;
    return;
  }


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
  novo->regComplemento = 0; // (T3) Inicializa o novo registador
  novo->pid_esperado = -1;

  // --- NOVO T3 ---
  novo->tam_memoria = 0; // Sera definido por so_carrega_programa
  novo->nome_executavel[0] = '\0'; // Sera definido por so_carrega_programa
  novo->tempo_termino_io_disco = 0;
  novo->num_page_faults = 0;

  //metricas
  novo->prioridade = 0.5;
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


  // PARTE 3 - T2
  int tempo_de_criacao;
  if (es_le(self->es, D_RELOGIO_INSTRUCOES, &tempo_de_criacao) == ERR_OK) {
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

  // 5. Retornar o PID do novo processo no registrador A do pai
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
  if (pid_alvo == 0) {
    pid_alvo = chamador->pid;
  }

  // 1. Encontrar o processo a ser morto na tabela
  int idx_alvo = -1;
  for (int i = 0; i < MAX_PROCESSOS; i++) {
    if (self->tabela_processos[i].pid == pid_alvo && self->tabela_processos[i].estado != TERMINADO) {
      idx_alvo = i;
      break;
    }
  }

  // Se não encontrou o processo, retorna erro
  if (idx_alvo == -1) {
    chamador->regA = -1; // Retorno de erro
    console_printf("SO: Tentativa de matar processo com PID %d, mas ele nao existe.", pid_alvo);
    return;
  }

  // 2. Mudar o estado do processo para TERMINADO
  processo_t *alvo = &self->tabela_processos[idx_alvo];

  //metricas
  int tempo_agora;
  es_le(self->es, D_RELOGIO_INSTRUCOES, &tempo_agora);
  alvo->tempo_termino = tempo_agora;

  //continuacao do 2.
  int pid_morto = alvo->pid; // Guarda o PID antes de o invalidar
  alvo->estado = TERMINADO;
  alvo->pid = -1; // Libera o PID

  // -----T3-----
  // 3. Libertar os recursos de memória do processo morto
  if (alvo->tabpag != NULL) {
    console_printf("SO: Libertando tabela de paginas do PID %d.", pid_morto);
    // TODO-T3: Percorrer a tabela e devolver os quadros para uma lista de livres
    
    // Liberta a estrutura da tabela de páginas
    tabpag_destroi(alvo->tabpag);
    alvo->tabpag = NULL;
  }

  console_printf("SO: Processo com PID %d terminado.", pid_alvo);

  // 2a. Desbloqueia processos que estavam à espera do processo que morreu
  for (int i = 0; i < MAX_PROCESSOS; i++) {
    processo_t *p = &self->tabela_processos[i];
    if (p->estado == BLOQUEADO && p->tipo_bloqueio == BLOQUEIO_ESPERA && p->pid_esperado == pid_morto) {
      
      p->estado = PRONTO;
      p->tipo_bloqueio = BLOQUEIO_NENHUM;
      p->pid_esperado = -1;
      p->regA = 0; // Retorna 0 (sucesso) para a chamada SO_ESPERA_PROC

      console_printf("SO: Processo %d desbloqueado pois processo %d terminou.", p->pid, pid_morto);
    }
  }

  // Se o processo se matou, o escalonador precisará escolher outro.
  // Forçamos o processo_atual_idx para -1 para garantir que o escalonador
  // não tente continuar com o processo que acabou de morrer.
  if (alvo->pid == chamador->pid) {
      self->processo_atual_idx = -1;
  }

  // 3. Retornar 0 (sucesso) no registrador A do processo chamador
  chamador->regA = 0;
}

// implementação da chamada se sistema SO_ESPERA_PROC
// espera o fim do processo com pid X
static void so_chamada_espera_proc(so_t *self)
{
  processo_t *chamador = &self->tabela_processos[self->processo_atual_idx];
  int pid_alvo = chamador->regX; // O PID a ser esperado está no registador X

  // 1. Validar o PID alvo
  // Não pode esperar por si mesmo
  if (pid_alvo == chamador->pid) {
    chamador->regA = -1; // Retorna erro
    console_printf("SO: Processo %d tentou esperar por si mesmo.", chamador->pid);
    return;
  }

  // O processo alvo tem de existir e não pode estar já terminado
  int idx_alvo = -1;
  for (int i = 0; i < MAX_PROCESSOS; i++) {
    if (self->tabela_processos[i].pid == pid_alvo && self->tabela_processos[i].estado != TERMINADO) {
      idx_alvo = i;
      break;
    }
  }

  // Se não encontrou um processo válido para esperar, retorna erro
  if (idx_alvo == -1) {
    chamador->regA = -1; // Retorna erro
    console_printf("SO: Processo %d tentou esperar por PID %d, que nao existe.", chamador->pid, pid_alvo);
    return;
  }

  // 2. Tudo válido, bloqueia o processo chamador
  chamador->estado = BLOQUEADO;
  chamador->vezes_bloqueado++; //metricas
  chamador->tipo_bloqueio = BLOQUEIO_ESPERA;
  chamador->pid_esperado = pid_alvo;

  // Força o escalonador a escolher outro processo
  self->processo_atual_idx = -1;

  console_printf("SO: Processo %d bloqueado, esperando pelo processo %d.", chamador->pid, pid_alvo);
}



// ---------------------------------------------------------------------
// TRATAMENTO DE FALTA DE PAGINA (T3) {{{1
// ---------------------------------------------------------------------

// Define o tempo de "transferencia" do disco (em instrucoes)
#define TEMPO_TRANSFERENCIA_DISCO 100

// Carrega uma pagina do "disco" (o arquivo .maq) para um quadro da memoria fisica
static void so_carrega_pagina_do_disco(so_t *self, processo_t *p, int pagina_virtual, int quadro_destino)
{
  // 1. Abre o programa
  programa_t *prog = prog_cria(p->nome_executavel);
  if (prog == NULL) {
    console_printf("SO: PF Handler: ERRO FATAL! Nao foi possivel abrir '%s'", p->nome_executavel);
    // Nao ha muito o que fazer aqui, o processo vai morrer
    self->erro_interno = true;
    return;
  }

  // 2. Calcula o offset no arquivo e o endereco fisico na memoria
  int end_virt_ini_prog = prog_end_carga(prog);
  int end_virt_pagina = pagina_virtual * TAM_PAGINA;
  int end_fis_quadro = quadro_destino * TAM_PAGINA;

  int offset_no_prog = end_virt_pagina - end_virt_ini_prog;

  console_printf("SO: PF Handler: Carregando pagina virtual %d (end %d) do disco...",
                   pagina_virtual, end_virt_pagina);
  console_printf("SO: PF Handler: ... para quadro fisico %d (end %d). Offset prog: %d",
                   quadro_destino, end_fis_quadro, offset_no_prog);

  // 3. Copia a pagina (byte a byte) do programa para a memoria fisica
  for (int i = 0; i < TAM_PAGINA; i++) {
    int end_virt_dado = end_virt_ini_prog + offset_no_prog + i;
    int end_fis_dado = end_fis_quadro + i;

    // O processo pode ser menor que a pagina inteira
    // E o endereco virtual do dado deve ser >= endereco de carga
    if (end_virt_dado >= end_virt_ini_prog && end_virt_dado < p->tam_memoria) {
      int dado = prog_dado(prog, end_virt_dado);
      mem_escreve(self->mem, end_fis_dado, dado);
    } else {
      // Preenche o resto da pagina com 0 (se for fora do .maq)
      mem_escreve(self->mem, end_fis_dado, 0);
    }
  }

  prog_destroi(prog);
}

// Encontra um quadro livre. Por enquanto, so incrementa o contador global
// Esta e a implementacao mais simples. Nao ha substituicao de pagina.
static int so_encontra_quadro_livre(so_t *self)
{
  // Gestao de memoria simples: apenas incrementa o contador de quadros
  
  // Precisamos saber o tamanho da memoria fisica.
  // Vamos assumir que mem_tam() existe (ela esta em memoria.h)
  int max_quadros = mem_tam(self->mem) / TAM_PAGINA;
  
  int quadro = self->quadro_livre;

  if (quadro >= max_quadros) {
     // MEMORIA CHEIA!
     return -1; // Sinaliza que precisa de substituicao
  }

  self->quadro_livre++;
  console_printf("SO: PF Handler: Alocando novo quadro fisico livre: %d", quadro);
  return quadro;
}


static void so_trata_falta_de_pagina(so_t *self)
{
  processo_t *p = &self->tabela_processos[self->processo_atual_idx];
  int end_falha = p->regComplemento; // Endereco virtual que causou a falha
  int pagina_virtual = end_falha / TAM_PAGINA;

  // 1. Verificar se o endereco e valido
  // O endereco de carga e salvo em programa_t, mas nao no processo_t
  // Vamos assumir que o endereco de carga e 0 para programas paginados
  // (O .maq do T3 e montado para 0).
  // A unica checagem e contra o tamanho maximo.
  if (end_falha < 0 || end_falha >= p->tam_memoria) {
    // Endereco invalido! E um "Segmentation Fault"
    console_printf("SO: PF Handler: ERRO! Endereco %d fora dos limites (0 - %d).",
                     end_falha, p->tam_memoria - 1);
    console_printf("SO: Matando processo %d por Segmentation Fault.", p->pid);
    
    // Forca o suicidio do processo
    p->regX = 0;
    so_chamada_mata_proc(self);
    return;
  }

  // 2. Endereco valido. E uma falta de pagina real.
  p->num_page_faults++; // Metrica
  console_printf("SO: PF Handler: Falta de pagina valida para PID %d, end %d (Pagina %d). PF total: %d",
                   p->pid, end_falha, pagina_virtual, p->num_page_faults);

  // 3. Encontrar um quadro livre na memoria fisica
  int quadro_destino = so_encontra_quadro_livre(self);

  // 4. Se nao ha quadros livres (quadro_destino == -1),
  //    precisamos rodar o algoritmo de substituicao.
  if (quadro_destino == -1) {
    // TODO-T3: Chamar o algoritmo de substituicao (FIFO ou LRU)
    console_printf("SO: PF Handler: MEMORIA CHEIA! Substituicao de pagina ainda nao implementada.");
    console_printf("SO: Matando processo %d.", p->pid);
    // Por enquanto, vamos matar o processo
    p->regX = 0;
    so_chamada_mata_proc(self);
    return;
  }

  // 5. Carregar a pagina da "memoria secundaria" (arquivo .maq)
  //    para o quadro fisico encontrado.
  so_carrega_pagina_do_disco(self, p, pagina_virtual, quadro_destino);

  // 6. Atualizar a tabela de paginas do processo
  tabpag_define_quadro(p->tabpag, pagina_virtual, quadro_destino);

  // 7. Simular o bloqueio por E/S de disco
  int tempo_agora;
  es_le(self->es, D_RELOGIO_INSTRUCOES, &tempo_agora);
  
  long tempo_termino_io;
  if (tempo_agora > self->tempo_disco_livre) {
    // Disco estava livre
    tempo_termino_io = tempo_agora + TEMPO_TRANSFERENCIA_DISCO;
  } else {
    // Disco estava ocupado, entra na "fila"
    tempo_termino_io = self->tempo_disco_livre + TEMPO_TRANSFERENCIA_DISCO;
  }
  self->tempo_disco_livre = tempo_termino_io;

  // 8. Bloquear o processo
  console_printf("SO: PF Handler: Bloqueando processo %d por E/S de disco ate %ld",
                   p->pid, tempo_termino_io);
  p->estado = BLOQUEADO;
  p->vezes_bloqueado++; //metricas
  p->tipo_bloqueio = BLOQUEIO_PAGINACAO;
  
  // Guarda o tempo de termino em nosso novo campo
  p->tempo_termino_io_disco = tempo_termino_io;
  
  // Forca o escalonador a escolher outro processo
  self->processo_atual_idx = -1;
}


// ---------------------------------------------------------------------
// CARGA DE PROGRAMA {{{1
// ---------------------------------------------------------------------

static int so_carrega_programa_na_memoria_fisica(so_t *self, programa_t *programa);
static int so_carrega_programa_na_memoria_virtual(so_t *self,
                                                  programa_t *programa,
                                                  processo_t *processo,
                                                  const char *nome_prog);

// carrega o programa na memória
// se processo_idx for NENHUM_PROCESSO, carrega o programa na memória física
//   senão, carrega na memória virtual do processo
// retorna o endereço de carga ou -1
static int so_carrega_programa(so_t *self, int processo_idx,
                               char *nome_do_executavel)
{
  console_printf("SO: carga de '%s'", nome_do_executavel);

  programa_t *programa = prog_cria(nome_do_executavel);
  if (programa == NULL) {
    console_printf("Erro na leitura do programa '%s'\n", nome_do_executavel);
    return -1;
  }

  int end_carga;
  if (processo_idx == NENHUM_PROCESSO) {
    end_carga = so_carrega_programa_na_memoria_fisica(self, programa);
  } else {
    processo_t *p = &self->tabela_processos[processo_idx];
    end_carga = so_carrega_programa_na_memoria_virtual(self, programa, p, nome_do_executavel);
  }

  prog_destroi(programa);
  return end_carga;
}

static int so_carrega_programa_na_memoria_fisica(so_t *self, programa_t *programa)
{
  int end_ini = prog_end_carga(programa);
  int end_fim = end_ini + prog_tamanho(programa);

  for (int end = end_ini; end < end_fim; end++) {
    if (mem_escreve(self->mem, end, prog_dado(programa, end)) != ERR_OK) {
      console_printf("Erro na carga da memória, endereco %d\n", end);
      return -1;
    }
  }

  console_printf("SO: carga na memória física %d-%d", end_ini, end_fim);
  return end_ini;
}

static int so_carrega_programa_na_memoria_virtual(so_t *self,
                                                  programa_t *programa,
                                                  processo_t *processo,
                                                  const char *nome_prog)
{
  // A lógica do T3 para carregar na memória virtual
  // Adaptada para usar a tabela de páginas do processo
  
  int end_virt_ini = prog_end_carga(programa);
  // o código abaixo só funciona se o programa iniciar no início de uma página
  if ((end_virt_ini % TAM_PAGINA) != 0) {
      console_printf("SO: Erro! Programa '%s' nao inicia no comeco de pagina.", nome_prog);
      return -1;
  }

  // --- ALTERACAO T3: Salvar metadados no PCB ---
  // Salva o nome e o tamanho total da memoria virtual no PCB
  // O tamanho e o endereco final + 1 (ou tamanho + endereco inicial)
  processo->tam_memoria = end_virt_ini + prog_tamanho(programa);
  strncpy(processo->nome_executavel, nome_prog, 99);
  processo->nome_executavel[99] = '\0';

  console_printf("SO: '%s' registrado para paginacao por demanda. Tamanho: %d bytes (EndVirt: %d a %d).",
                   nome_prog, processo->tam_memoria - end_virt_ini, end_virt_ini, processo->tam_memoria - 1);
  

  /*
  int end_virt_fim = end_virt_ini + prog_tamanho(programa) - 1;
  int pagina_ini = end_virt_ini / TAM_PAGINA;
  int pagina_fim = end_virt_fim / TAM_PAGINA;
  int n_paginas = pagina_fim - pagina_ini + 1;

  // Aloca quadros de memória física para estas páginas
  // (Usando a gestão de memória simples do T3, que apenas incrementa)
  int quadro_ini = self->quadro_livre;
  int quadro_fim = quadro_ini + n_paginas - 1;
  // TODO-T3: Verificar se quadro_fim ultrapassa a memória física!
  
  console_printf("SO: Mapeando %d paginas (V:%d-%d) para quadros (F:%d-%d)",
                 n_paginas, pagina_ini, pagina_fim, quadro_ini, quadro_fim);

  // Mapeia as páginas na tabela de páginas do processo
  for (int i = 0; i < n_paginas; i++) {
    tabpag_define_quadro(processo->tabpag, pagina_ini + i, quadro_ini + i);
  }
  self->quadro_livre = quadro_fim + 1;

  // Carrega o programa na memória física, quadro a quadro
  int end_fis_ini = quadro_ini * TAM_PAGINA;
  int end_fis = end_fis_ini;
  for (int end_virt = end_virt_ini; end_virt <= end_virt_fim; end_virt++) {
    if (mem_escreve(self->mem, end_fis, prog_dado(programa, end_virt)) != ERR_OK) {
      console_printf("Erro na carga da memória, end virt %d fís %d\n", end_virt,
                     end_fis);
      // TODO-T3: Desfazer o mapeamento em caso de erro!
      return -1;
    }
    end_fis++;
  }
  
  console_printf("SO: carga na memória virtual V%d-%d F%d-%d npag=%d",
                 end_virt_ini, end_virt_fim, end_fis_ini, end_fis - 1, n_paginas);

  */
  return end_virt_ini;
}


// ---------------------------------------------------------------------
// ACESSO À MEMÓRIA DOS PROCESSOS {{{1
// ---------------------------------------------------------------------

// copia uma string da memória do simulador para o vetor str.
// retorna false se erro (string maior que vetor, valor não char na memória,
//   erro de acesso à memória)
// t2: deveria verificar se a memória pertence ao processo
static bool so_copia_str_do_processo(so_t *self, int tam, char str[tam],
  int end_virt, int processo_idx)
{
if (processo_idx == NENHUM_PROCESSO) return false;

// Define *temporariamente* a MMU para a tabela de páginas
// do processo de onde queremos ler
processo_t *p = &self->tabela_processos[processo_idx];
tabpag_t *tabpag_original = self->tabela_processos[self->processo_atual_idx].tabpag;
mmu_define_tabpag(self->mmu, p->tabpag);

bool sucesso = true;
for (int indice_str = 0; indice_str < tam; indice_str++) {
int caractere;
// Lê usando o modo usuário para forçar a tradução de endereços
if (mmu_le(self->mmu, end_virt + indice_str, &caractere, usuario) != ERR_OK) {
sucesso = false;
break;
}
if (caractere < 0 || caractere > 255) {
sucesso = false;
break;
}
str[indice_str] = caractere;
if (caractere == 0) {
break; // Sucesso, string copiada
}
}

// Restaura a tabela de páginas original (do processo que estava executando)
mmu_define_tabpag(self->mmu, tabpag_original);

return sucesso;
}

// vim: foldmethod=marker
