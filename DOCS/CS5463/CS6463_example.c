
Arquivo de cabeçalho
#include <REG52.h>
#include <string.h>
#include <intrins.h>
#define uint  unsigned int
#define uchar caractere não assinado

typedef bit  bool;
typedef unsigned char uint8; /* definido para variável inteira de 8 bits sem sinal */
typedef signed char int8; /* definido para variável inteira de 8 bits assinada */
typedef unsigned int uint16; /* definido para variável inteira de 16 bits sem sinal */
typedef assinado int int16; /* definido para variável inteira de 16 bits assinada */
typedef não assinado longo uint32; /* definido para variável inteira de 32 bits sem sinal */
typedef assinado long int32; /* definido para variável inteira de 32 bits assinada */
typedef float fp32; /* variável de ponto flutuante de precisão única (32 bits) */

/*****************************************************************************/
sbit LCM_cs = P0^0; RS
sbit LCM_std = P0^1; SID
sbit LCM_sclk = P0^2; SCLK
sbit LCM_psb = P2^3; H=paralelo; L = porta serial;

char a, b, c;
char aa,bb,cc,dd,ee;
char i,q,T=125;
código uchar tab1[]={
"Voltagem: "
"Poder: "
"Atual: "
"Temperatura: "
};
/****************************************************************************/

uchar r[3]={0x00,0x00,0x00};

/******************************************************************
Definição de interface
******************************************************************/
sbit SCLK=P2^2; Ck
sbit MOSI=P2^1; DI
bit MISO=P2^0; FAZER
bit INT=P3^2;
bit CS=P2^3;
sbit RST=P2^4; RST

#define CS5463_VScale 525 // Calcule a relação de tensão, 220V * 250mv / 110mv = 500V
#define CS5463_IScale (250/10) // Calcule o índice atual

uint8 estático RX_Buff[4]; CS5463 Buffer de leitura e gravação
uint8 sta; Status do chip

#define READ_MASK 0xBF //O código de blindagem ao ler o registro, que é o mesmo que o endereço (de gravação)
#define CMD_SYNC0 0XFE //Finalizar a reinicialização da porta serial
#define CMD_SYNC1 0XFF //Iniciar reinicialização da porta serial
#define REG_CONFR 0x40 //Configuração
#define REG_CYCCONT 0x4A //Número de transformações A/D em um período de cálculo
#define REG_STATUSR 0x5E //Status
#define REG_MODER 0x64 // modo de operação
#define REG_MASKR 0x74 //Mascaramento de interrupção
#define REG_CTRLR 0x78 // Controle
#define CMD_STARTC 0XE8 // Executar ciclos de cálculo contínuos

#define REG_VRMSR           0X18          //VRMS
#define REG_IRMSR           0X16          //IRMS
#define REG_Pactive 0X14 //Ativo

relé sbit = P3^7; Definição do pino de corte
uint32 CurrentCtl; Variável global atual

/*************************************************************
** Nome da função: uDelay
** Função de função: atraso de tempo
** Parâmetros da função: J
** Valor retornado: Nenhum
** Criado: 2009-4-23
** Primeira modificação: nenhuma
**************************************************************/
vazio estático uDelay(uint8 j)
{
  uint8 i;
for(; j>0; j--)
{ for(i=0; i<255; i--)
                {
                ;
                }
        }
}
/*************************************************************
** Nome da função: CS5463CMD
** Função de função: função de comando CS5463
** Parâmetros de função: Nenhum
** Criado: 2009-9-14
** Primeira modificação: nenhuma
**************************************************************/
vazio estático CS5463CMD(uint8 cmd)
{
uint8 i;
SCLK = 1;
CS = 0;
i = 0;
enquanto(i<8)
{
  uDelay(50);
        SCLK = 0;
        if(cmd&0x80)MOSI = 1;
senão MOSI = 0;
        uDelay(50);
SCLK = 1; Na borda ascendente do relógio, os dados são gravados no CS5463
        cmd <<= 1;
        i++;
}
uDelay(50);
CS = 1;
}
/*************************************************************
** Nome da função: CS5463WriteReg
** Função de função: CS5463 escreva a função do registro
** Parâmetros de função: Nenhum
** Criado: 2009-9-14
** Primeira modificação: nenhuma
**************************************************************/
void CS5463WriteReg(uint8 addr,uint8 *p)
{
uint8 i,j;
uint8 dat;
SCLK = 1;
CS = 0;
i = 0;
enquanto(i<8)
{
          uDelay(50);
        SCLK = 0;
        if(addr&0x80)MOSI = 1;
senão MOSI = 0;
        uDelay(50);
SCLK = 1; Na borda ascendente do relógio, os dados são gravados no CS5463
endereço <<= 1;
        i++;
}
j = 0;
enquanto(j<3)
{
          dat = *(p+j);
        i = 0;
enquanto(i<8)
        {
                  uDelay(50);
                SCLK = 0;
                if(dat&0x80)MOSI = 1;
senão MOSI = 0;
                uDelay(50);
SCLK = 1; Na borda ascendente do relógio, os dados são gravados no CS5463
                dat <<= 1;
                i++;
        }
        j++;
}
uDelay(50);
CS = 1;
}
/*************************************************************
Nome da função: CS5463ReadReg
** Função de função: CS5463 leia a função de registro
** Parâmetros de função: Nenhum
** Criado: 2009-9-14
** Primeira modificação: nenhuma
**************************************************************/
void CS5463ReadReg(uint8 addr,uint8 *p)
{
uint8 i,j;
uint8 dat;
SCLK = 0;
CS = 0;
addr &= READ_MASK;
i = 0;
enquanto(i<8)
{
          uDelay(50);
        SCLK = 0;
        if(addr&0x80)MOSI = 1;
senão MOSI = 0;
        uDelay(50);
        SCLK = 1;
endereço <<= 1; Na borda ascendente do relógio, os dados são gravados no CS5463
        i++;
}
uDelay(50);
MOSI = 1;
j = 0;
enquanto(j<3)
{
        i = 0;
        dat = 0;
enquanto(i<8)
        {
                if(i==7)MOSI = 0;
senão MOSI = 1;
                SCLK = 0;
                uDelay(50);
                dat <<= 1;                                                
                if(MISO)dat |= 0x01;
                else        dat &= 0xFE;
                SCLK = 1;
                uDelay(50);                                                         
                i++;
        }
        *(p+j) = dat;
        j++;
}
MOSI = 1;
CS = 1;
}
/*************************************************************************************************
CS5463 Funções do aplicativo
*************************************************************************************************/
/*************************************************************
** Nome da função: CS5463Init
** Função de função: CS5463 redefinir e inicializar a função
** Parâmetros de função: Nenhum
** Criado: 2009-9-14
** Primeira modificação: nenhuma
**************************************************************/
bit CS5463_Init(void)           //
{
RST = 0;
uDelay(200);
RST = 1;
uDelay(100);
//----------------------
Enviar uma sequência de sincronização
RX_Buff[0] = CMD_SYNC1;
RX_Buff[1] = CMD_SYNC1;
RX_Buff[2] = CMD_SYNC0;
CS5463WriteReg(CMD_SYNC1,RX_Buff); #define CMD_SYNC1 0XFF //Iniciar reinicialização da porta serial
//----------------------
Inicialização -- Configurar registros
A compensação de fase é PC [6: 0] = [0000000];
O ganho atual do canal é Igain=10;
EWA=0;
A interrupção INT está ativa-baixa IMODE: IINV=[00]
iCPU=0
K[3:0]=[0001]
RX_Buff[0] = 0x00;                                                
RX_Buff[1] = 0x00;
RX_Buff[2] = 0x01;
CS5463WriteReg(REG_CONFR,RX_Buff); #define REG_CONFR 0x40 // Configuração
//----------------------
Inicializar -- Operar registros
RX_Buff[0] = 0x00; B0000_0000;
RX_Buff[1] = 0x00; B0000_0000;
RX_Buff[2] = 0x60; B0110_0000;
CS5463WriteReg(REG_MODER,RX_Buff); #define REG_MODER 0x64 // modo de operação
//----------------------
Inicialização - Registro de calibração de polarização CA atual
RW24XX(RX_Buff,3,EE_IACBIAS,0xA1);
CS5463WriteReg(REG_IACOFF,RX_Buff);
//----------------------
Inicializar - Registro de calibração de ganho de corrente
RW24XX(RX_Buff,3,EE_IACGAIN,0xA1);
CS5463WriteReg(REG_IGN,RX_Buff);
//----------------------
Inicialização - Registro de calibração de polarização CA de tensão
RW24XX(RX_Buff,3,EE_VACBIAS,0xA1);
CS5463WriteReg(REG_VACOFF,RX_Buff);
//----------------------
Inicializar - Registro de calibração de ganho de tensão
RW24XX(RX_Buff,3,EE_VACGAIN,0xA1);
CS5463WriteReg(REG_VGN,RX_Buff);
//----------------------
RX_Buff[0] = 0x00;
RX_Buff[1] = 0x0F;
RX_Buff[2] = 0xA0; #define REG_CYCCONT 0x4A //Número de transformações A/D em um período de cálculo
CS5463WriteReg(REG_CYCCONT,RX_Buff); Inicializar --CYCLE COUNT register, 4000
//----------------------
Inicialização - Registros de frequência de pulso
RX_Buff[0] = 0x00;
RX_Buff[1] = 0x34;
RX_Buff[2] = 0x9C;
CS5463WriteReg(REG_PULRATE,RX_Buff);
//----------------------
RX_Buff[0] = 0xFF;
RX_Buff[1] = 0xFF;
RX_Buff[2] = 0xFF;
CS5463WriteReg(REG_STATUSR,RX_Buff); Inicializar --status registra #define REG_STATUSR 0x5E //status
//----------------------
RX_Buff[0] = 0x80; A corrente, a tensão e a potência são interrompidas após a conclusão da medição
RX_Buff[1] = 0x00;
RX_Buff[2] = 0x80; A medição de temperatura é interrompida
CS5463WriteReg(REG_MASKR,RX_Buff); Initialize - Registro de blindagem de interrupção #define REG_MASKR 0x74 // Blindagem de interrupção
//----------------------
RX_Buff[0] = 0x00;
RX_Buff[1] = 0x00;
RX_Buff[2] = 0x00;
CS5463WriteReg(REG_CTRLR,RX_Buff); Initialize -- Registradores de controle #define REG_CTRLR 0x78 //Control
//----------------------
CS5463CMD(CMD_STARTC); Iniciar uma conversão contínua #define CMD_STARTC 0XE8 //Executar um ciclo de cálculo contínuo
CS5463_Status = 0; Inicializar o status do processo da tarefa
Load_Status = 0;
CS5463_CrmsSmallCunt = 0;
CS5463_CrmsOverCunt = 0;
return(1); Contanto que você execute essas etapas, true 1 será retornado
}
/*************************************************************
** Nome da função: CS5463_ResetStatusReg
** Função de função: Função de registro de status de redefinição
** Parâmetros de função: Nenhum
** Criado: 2009-9-15
** Primeira modificação: nenhuma
**************************************************************/
vazio estático CS5463_ResetStatusReg(vazio)
{
RX_Buff[0] = 0xFF;
RX_Buff[1] = 0xFF;
RX_Buff[2] = 0xFF;
CS5463WriteReg(0x5E,RX_Buff); Redefinir status registra #define REG_STATUSR 0x5E //status
}
/*************************************************************
** Nome da função: CS5463_GetStatusReg
** Função de função: Leia a função de registro de status
** Parâmetros de função: Nenhum
** Criado: 2009-9-15
** Primeira modificação: nenhuma
**************************************************************/
estático uint8 CS5463_GetStatusReg(void)
{
uint8 sta=0;
CS5463ReadReg(0x1E,RX_Buff); O que é 1E? Registros de status
if(RX_Buff[0]&0x80) //Detecção: Se as medições de corrente, tensão e potência foram concluídas
{
Detecte se a corrente/tensão está fora da faixa
Detecte se a corrente RMS/tensão RMS/energia elétrica está fora da faixa
if((RX_Buff[0]&0x03)|| (RX_Buff[1]&0x70))
        {
CS5463_ResetStatusReg(); Redefinir o registro de status
        }
mais
        {
sta |= 0x01; B0000_0001; //
        }
}

if(RX_Buff[2]&0x80) //Detecção: se a medição de temperatura está completa
{
sta |=0x02; B0000_0010;
}
retorno(sta);
}  

void DelayM(unsigned int a) //Função de atraso 1MS/time
{
caractere não assinado i;
                while( --a != 0)
       {               
for(i = 0; I < 125; i++); Um; Indica uma instrução vazia na qual a CPU ociosa é adicionada de 0 a 125
                }                                      
}

void Delay(int num) //Função de atraso
{
enquanto(num--);
}

/******************************************************************************/
Instrução de gravação ou dados (0, Instrução) (1, Dados)
void LCM_WriteDatOrCom (bit dat_comm, conteúdo uchar)
{
uchar a, i, j;
Atraso(50);
a = conteúdo;
LCM_cs = 1;
LCM_sclk = 0;
LCM_std = 1;
for(i=0; i<5; i++)
  {
LCM_sclk = 1;
LCM_sclk = 0;
  }
LCM_std = 0;
LCM_sclk = 1;
LCM_sclk = 0;
se(dat_comm)
LCM_std = 1; dados
mais
LCM_std = 0; comando
LCM_sclk = 1;
LCM_sclk = 0;
LCM_std = 0;
LCM_sclk = 1;
LCM_sclk = 0;
for(j=0; j<2; j++)
  {
for(i=0; i<4; i++)
    {
      a=a<<1;
      LCM_std=CY;
LCM_sclk = 1;
LCM_sclk = 0;
    }
LCM_std = 0;
for(i=0; i<4; i++)
    {
LCM_sclk = 1;
LCM_sclk = 0;
    }
  }
}
/*********************************************************************************/


/*****************************************************************************/
Inicializar o LCM
void LCM_init(void)         
{
LCM_psb = 0;
LCM_WriteDatOrCom (0,0x30); /*30---Ação de comando básica*/
LCM_WriteDatOrCom (0,0x01); /*Limpe a tela, o ponteiro de endereço aponta para 00H*/
Atraso (100);
LCM_WriteDatOrCom (0,0x06); /*Direção do movimento do cursor*/
LCM_WriteDatOrCom(0,0x0c); /*No visor, cursor desligado*/
}

void chn_disp (código uchar *chn) // Exibe 4 linhas de ponteiros
{
  uchar i,j;
  LCM_WriteDatOrCom  (0,0x30);         //        
  LCM_WriteDatOrCom  (0,0x80);         //
para (j=0; j<4; j++)
  {
para (i=0; i<16; i++)
    LCM_WriteDatOrCom  (1,chn[j*16+i]);
  }
}
/*****************************************************************************/
Função de tela clara
Vazio LCM_clr(Vazio)
{
  LCM_WriteDatOrCom (0,0x30);
LCM_WriteDatOrCom (0,0x01); /*Limpe a tela, o ponteiro de endereço aponta para 00H*/
Atraso (180);
}
/*****************************************************************************/
Envie uma sequência de 64 caracteres ou menos para o LCM.
Aplicação: LCM_WriteString ("Olá!") );
void LCM_WriteString(caractere não assinado *str)
{
                while(*str != '\0')
       {
                        LCM_WriteDatOrCom(1,*str++);
        }
                *str = 0;        
}

/*************************************************************
** Nome da função: CS5463_GetCurrentRMS
** Função de função: leia a função RMS atual
** Parâmetros de função: Nenhum
** Criado: 2009-9-15
** Primeira modificação: nenhuma
**************************************************************/
vazio estático CS5463_GetCurrentRMS(vazio)
{
fp32 G = 0,5,resultado;
uint32 temp1;
uint8 temp,i,j;
CS5463ReadReg(REG_IRMSR,RX_Buff); Ler o valor atual do RMS
SndCom1Data(RX_Buff,3);
i = 0;
resultado = 0;
enquanto(i<3)
{
          temp = RX_Buff[i];                                          
        j = 0;
enquanto(j<8)
        {
                 if(temp&0x80)
                {
resultado += G;
                }
                temp <<= 1;
                j++;
                G = G/2;        
        }
        i++;
}
resultado = resultado*CS5463_IScale; I_Coff; Calcule o valor atual Não por enquanto
resultado *= 1000; Unidade: mA (miliamperes) 12345ma
temp1 = (uint32)resultado;
CurrentCtl = temp1;

LCM_WriteDatOrCom  (0,0x94);
        aa=        temp1/10000;
        if(aa==0)
LCM_WriteDatOrCom(1'');
mais
                LCM_WriteDatOrCom(1,aa+0x30);
        bb=        (temp1%10000)/1000;
        if((aa==0)&&(bb==0))
LCM_WriteDatOrCom(1'');
mais
                LCM_WriteDatOrCom(1,bb+0x30);
        cc=(temp1%1000)/100;
        if((aa==0)&&(bb==0)&&(cc==0))
LCM_WriteDatOrCom(1'');
mais
LCM_WriteDatOrCom (1, cc + 0x30);
LCM_WriteDatOrCom (1.0x2e); Ponto decimal Nenhum ponto decimal é necessário
        dd=        (temp1%100)/10;
        LCM_WriteDatOrCom(1,dd+0x30);
        ee=temp1%10;
        LCM_WriteDatOrCom(1,ee+0x30);
        LCM_WriteString(" mA");

}


/*************************************************************
** Nome da função: CS5463_GetPactiveRMS
** Função de função: Leia a função de potência ativa
** Parâmetros de função: Nenhum
** Criado: 2009-9-15
** Primeira modificação: nenhuma
**************************************************************/
vazio estático CS5463_GetPactiveRMS(vazio)
{
fp32 G = 1,0,resultado;
uint8 temp,i,j;
uint32 temp1;
CS5463ReadReg(0x14,RX_Buff); Leia o REG_Pactive de potência ativa
SndCom1Data(RX_Buff,3);
temp = RX_Buff[0];
if(temp&0x80) // Se o número for negativo, o código original é calculado
{
RX_Buff[0] = ~RX_Buff[0]; Originalmente, foi negado +1, mas aqui por causa da precisão, não é +1
        RX_Buff[1] = ~RX_Buff[1];
        RX_Buff[2] = ~RX_Buff[2];                        
}
i = 0;
resultado = 0;
enquanto(i<3)
{
          temp = RX_Buff[i];                                          
        j = 0;
enquanto(j<8)
        {
                 if(temp&0x80)
                {
resultado += G;
                }
                temp <<= 1;
                j++;
                G = G/2;        
        }
        i++;
}
resultado = resultado*P_Coff; Potência calculada em W (watts)
resultado = Vrms*Irms; Calcule a potência diretamente
resultado = resultado*13125;
temp1 = (uint32)resultado;

LCM_WriteDatOrCom (0,0x8C); 26W 12345W
        aa=        temp1/10000;
        if(aa==0)
LCM_WriteDatOrCom(1'');
mais
                LCM_WriteDatOrCom(1,aa+0x30);
        bb=        (temp1%10000)/1000;
        if((aa==0)&&(bb==0))
LCM_WriteDatOrCom(1'');
mais
                LCM_WriteDatOrCom(1,bb+0x30);
        cc=(temp1%1000)/100;
        if((aa==0)&&(bb==0)&&(cc==0))
LCM_WriteDatOrCom(1'');
mais
LCM_WriteDatOrCom (1, cc + 0x30);
LCM_WriteDatOrCom (1.0x2e); Ponto decimal Nenhum ponto decimal é necessário
        dd=        (temp1%100)/10;
        LCM_WriteDatOrCom(1,dd+0x30);
        ee=temp1%10;
        LCM_WriteDatOrCom(1,ee+0x30);
        LCM_WriteString(" W");

}
/*************************************************************
** Nome da função: CS5463_GetPowerFactor
** Função de função: Leia a função do fator de potência
** Parâmetros de função: Nenhum
** Criado: 2009-11-02
** Primeira modificação: nenhuma
**************************************************************
vazio estático CS5463_GetPowerFactor(vazio)
{
fp32 G = 1,0,resultado;
uint8 temp,i,j;
uint32 temp1;
CS5463ReadReg(0x32,RX_Buff); Leia o fator de potência
SndCom1Data(RX_Buff,3);
temp = RX_Buff[0];
if(temp&0x80) // Se o número for negativo, o código original é calculado
{
RX_Buff[0] = ~RX_Buff[0]; Originalmente, foi negado +1, mas aqui por causa da precisão, não é +1
        RX_Buff[1] = ~RX_Buff[1];
        RX_Buff[2] = ~RX_Buff[2];                        
}
i = 0;
resultado = 0;
enquanto(i<3)
{
          temp = RX_Buff[i];                                          
        j = 0;
enquanto(j<8)
        {
                 if(temp&0x80)
                {
resultado += G;
                }
                temp <<= 1;
                j++;
                G = G/2;        
        }
        i++;
}
resultado *= 10000;
temp1 = (uint32)resultado;

}


/*************************************************************
** Nome da função: CS5463_GetTemperature
** Função de função: Leia a função de temperatura
** Parâmetros de função: Nenhum
** Criado: 2009-11-03
** Primeira modificação: nenhuma
**************************************************************/
vazio estático CS5463_GetTemperature(vazio) // a energia de temperatura é mostrada PT2017-2-12
{
fp32 G = 128,resultado;
uint8 temp,i,j,pn=0;
uint32 temp1;
CS5463ReadReg(0x26,RX_Buff); Leia a temperatura Sim, leia a temperatura aqui
SndCom1Data(RX_Buff,3);
temp = RX_Buff[0];
if(temp&0x80) // Se o número for negativo, o código original é calculado
{
pn = 1; Sinalizadores de número negativo
RX_Buff[0] = ~RX_Buff[0]; Originalmente, foi negado +1, mas aqui por causa da precisão, não é +1
        RX_Buff[1] = ~RX_Buff[1];
        RX_Buff[2] = ~RX_Buff[2];                        
}
i = 0;
resultado = 0; Esse valor é um número de ponto flutuante, então nós o limpamos para zero e, em seguida, adicionamos os pesos de 0,5 um por um
enquanto(i<3)
{
temp = RX_Buff[i]; Embora esta matriz seja definida como 4 bytes, na verdade ela usa Buff[0] Buff[1] RX_Buff[2]
        j = 0;
enquanto(j<8)
        {
                 if(temp&0x80)
                {
resultado += G; Adicione o peso de 0,5
                }
                temp <<= 1;
                j++;
                G = G/2;        
        }
        i++;
}
if(result<128) //Sim, este resultado é -127.128 O valor do ponto flutuante da temperatura foi obtido aqui e tem no máximo 3 dígitos? Há também pontos decimais
{
resultado *= 100;
temp1 = (uint32)resultado; Sim, aqui está, por exemplo, como exibir 12523 -----> 125.23? Como desconectar a partir de 8A

LCM_WriteDatOrCom (0,0x9C); Mostrar a posição inicial na linha 4
                aa=        temp1/ 10000;
                if(pn==1)
                        LCM_WriteDatOrCom(1,'-');        
mais
                        LCM_WriteDatOrCom(1,'+');        
                        
                bb=temp1/1000- aa*10;
                LCM_WriteDatOrCom(1,bb+0x30);

                cc=        temp1/100- aa*100-bb*10;
LCM_WriteDatOrCom (1, cc + 0x30);
LCM_WriteDatOrCom (1.0x2e); //"."
                dd=        (temp1%100)/10;
                LCM_WriteDatOrCom(1,dd+0x30);
                ee=temp1%10;
                LCM_WriteDatOrCom(1,ee+0x30);
LCM_WriteString("°C");

}
}
/*************************************************************
** Nome da função: CS5463_GetVoltRMS
** Função de função: Leia a função do valor efetivo da tensão
** Parâmetros de função: Nenhum
** Criado: 2009-9-15
** Hora da primeira modificação: 2009-9-23, coeficiente de tensão modificado (verificação necessária)
** Segunda modificação
** Terceira modificação
**************************************************************/
vazio estático CS5463_GetVoltRMS(void) //PT2017-2-12 A exibição de tensão está OK
{
float G = 0,5,result; typedef float fp32; É um tipo de ponto flutuante
int temp1; int
uint8 temp,i,j; byte
CS5463ReadReg(REG_VRMSR,RX_Buff); Leia o #define REG_VRMSR 0x58 de tensão RMS
SndCom1Data(RX_Buff,3);
i = 0;
resultado = 0;
enquanto(i<3)
{
          temp = RX_Buff[i];                                          
        j = 0;
enquanto(j<8)
        {
                 if(temp&0x80)
                {
resultado += G;
                }
                temp <<= 1;
                j++;
                G = G/2;        
        }
        i++;                                                               
} // Em 220, a tensão de amostragem é de 78mV
resultado = resultado*CS5463_VScale; V_Coff; O valor de tensão calculado é 220V * 250mV / (110mv / 1,414) = 704,8V, que pode ser temporariamente desativado
if(result<=100)retornar; Se a leitura medida voltage for inferior a 100V, confirme se a leitura está incorreta
resultado *= 100; A unidade é mV (milivolt) 12345mv 5 bits como você exibe
temp1 = (uint32)resultado;

        LCM_WriteDatOrCom  (0,0x84);
        aa=        temp1/10000;
        LCM_WriteDatOrCom(1,aa+0x30);
        bb=        (temp1%10000)/1000;
        LCM_WriteDatOrCom(1,bb+0x30);
        cc=(temp1%1000)/100;
LCM_WriteDatOrCom (1, cc + 0x30);
LCM_WriteDatOrCom (1.0x2e);
        dd=        (temp1%100)/10;
        LCM_WriteDatOrCom(1,dd+0x30);
        ee=temp1%10;
        LCM_WriteDatOrCom(1,ee+0x30);
        LCM_WriteString(" V");

}

void main() //Toda a função do programa começa a partir daqui
{
  CS5463_Init();
LCM_init(); Inicialize o LCD
LCM_clr(); Limpe a tela
chn_disp(tab1); Mostre a palavra de boas-vindas
AtrasoM (500); Aguarde 3 segundos para a exibição
Relé = 1;
  
enquanto(1)
          {
if(INT)break; Verifique se há sinais de interrupção

sta = CS5463_GetStatusReg(); Detecte a causa da interrupção
if(0x01==(sta&0x01)) // Ler corrente e tensão
                {        
CS5463Monitor_Cunt = 0; Se houver uma interrupção, indicando que o chip está funcionando normalmente, limpe o temporizador de monitoramento
CS5463_ResetStatusReg(); Limpar a bandeira
CS5463_GetVoltRMS(); Obtenha a voltagem
CS5463_GetCurrentRMS(); Obter o atual
CS5463_GetPactiveRMS(); Obtenha energia
CS5463_GetPowerFactor(); Obtenha o fator de potência
if(0x02==(sta&0x02)) // Ler a temperatura
                        {        
CS5463_GetVoltRMS(); Obtenha a voltagem
CS5463_GetTemperature(); A temperatura não precisa ser lida com muita frequência, por isso é lida junto com a corrente e a tensão

CS5463_Init(); Reinicializar o chip
                        }                                                
SndCom1Data(MeasureData,16);
                }
if(CurrentCtl > 5000)//Se a corrente for maior que os 5A necessários, a energia será desligada
…………
…………
