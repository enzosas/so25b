# t2 - implementação de processos

## Parte I

Entenda as mudanças no código e o funcionamento do SO.

Faça uma implementação inicial de processos:
- crie um tipo de dados que é uma estrutura que contém as informações a respeito de um processo
- crie uma tabela de processos, que é um vetor dessas estruturas
- inicialize a primeira entrada nessa tabela (o primeiro processo) na criação do init
- crie uma variável que designe o processo em execução. Faça de tal forma que tenha suporte a não ter nenhum processo em execução
- altere `so_salva_estado_da_cpu` e `so_despacha` para salvar o estado do processador na tabela de processos (na entrada correspondente ao processo em execução) e para recuperar o estado do processador a partir da tabela
- implemente a função do escalonador (`so_escalona`). Ela escolhe o próximo processo a executar (altera a variável que designa o processo em execução). Pode ser bem simples: se o processo que estava em execução estiver pronto continua sendo executado e se não, escolhe o primeiro que encontrar na tabela que esteja pronto. 
- implemente as chamadas de criação e morte de processos
- altere as chamadas de E/S, para usar um terminal diferente dependendo do pid do processo
- o pid do processo não é a mesma coisa que sua entrada na tabela: quando um processo termina sua entrada na tabela pode ser reutilizada por outro processo, o pid não, é uma espécie de número de série do processo.

## Parte II

Na parte I, um processo não bloqueia, se ele não está morto, ele pode executar.
Nesta parte, vamos implementar o bloqueio de processos e eliminar a espera ocupada na E/S.
- nas chamadas de E/S, se o dispositivo não estiver pronto, o SO deve bloquear o processo e não realizar a E/S; se o dispositivo estiver pronto, ele realiza a E/S e não bloqueia, como na parte I.
- na função que trata de pendências, o SO deve verificar o estado dos dispositivos que causaram bloqueio e realizar operações pendentes e desbloquear processos se for o caso
- implemente a chamada de sistema SO_ESPERA_PROC, que bloqueia o processo chamador até que o processo que ele identifica na chamada tenha terminado. Se o processo esperado não existe ou se for o próprio processo chamador, retorna um erro para o processo, não bloqueia ele esperando algo que não vai acontecer. Quando tratar a morte de um processo, o SO deve verificar se há alguém esperando por esse acontecimento.

## Parte III

Implemente um escalonador preemptivo *round-robin* (circular):
- os processos prontos são colocados em uma fila
- o escalonador sempre escolhe o primeiro da fila
- quando um processo fica pronto (é criado ou desbloqueia), vai para o final da fila
- se terminar o *quantum* de um processo, ele é colocado no final da fila (preempção)

O *quantum* é definido como um múltiplo do intervalo de interrupção do relógio (em outras palavras, o *quantum* equivale a tantas interrupções). Quando um processo é selecionado para executar, tem o direito de executar durante o tempo de um quantum. Uma forma simples de implementar isso é ter uma variável do SO, controlada pelo escalonador, que é inicializada com o valor do quantum (em interrupções) quando um processo diferente do que foi interrompido é selecionado. Cada vez que recebe uma interrupção do relógio, decrementa essa variável. Quando essa variável chega a zero, o processo corrente é movido para o final da fila, se não tiver bloqueado.

Implemente um segundo escalonador, semelhante ao circular: os processos têm um quantum, e sofrem preempção quando esse quantum é excedido. Os processos têm também uma prioridade, e o escalonador escolhe o processo com maior prioridade entre os prontos.
A prioridade de um processo é calculada da seguinte forma:
- quando um processo é criado, recebe prioridade 0,5
- quando um processo perde o processador (porque bloqueou ou porque acabou seu quantum), a prioridade do processo é calculada como `prio = (prio + t_exec/t_quantum) / 2`, onde `t_exec` é o tempo desde que ele foi escolhido para executar e `t_quantum` é o tempo do quantum. O `t_exec` é o quantum menos o valor da variável que o escalonador decrementa a cada interrupção.
O valor da prioridade é uma espécie de média do percentual do quantum que o processo usou a cada vez que executou. Quanto menor esse valor, maior deve ser a prioridade do processo.

O SO deve manter algumas métricas, que devem ser apresentadas no final da execução do SO (quando o init morrer):
- número de processos criados
- tempo total de execução
- tempo total em que o sistema ficou ocioso (todos os processos bloqueados)
- número de interrupções recebidas de cada tipo
- número de preempções
- tempo de retorno de cada processo (diferença entre data do término e da criação)
- número de preempções de cada processo
- número de vezes que cada processo entrou em cada estado (pronto, bloqueado, executando)
- tempo total de cada processo em cada estado (pronto, bloqueado, executando)
- tempo médio de resposta de cada processo (tempo entre desbloquear e ser escalonado)

Os tempos acima são medidos em número de instruções (medidas pelo relógio).

Gere um relatório de execuções do sistema em diferentes configurações.


## Alterações / Respostas

- 2set - alterado o Makefile para explicitar o shell a usar (parece que tava com problemas
   no WSL)
- 2set - criado "compila" com os comandos de compilação, caso ainda dê problema com o Makefile
