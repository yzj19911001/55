/********************************************************************************************************/
/*** \文件名    socket.h															  *******************/
/*** \作者      李玉株																  *******************/
/*** \时间      2013年9月5日														  *******************/
/*** \文件说明  1、部分原代码来源于WIZnet官网 1.2.0版本									  ***************/
/***   			2、该文件定义了socket文件的一些函数说明。                                     ***********/
/********************************************************************************************************/
#ifndef	_SOCKET_H_
#define	_SOCKET_H_	
/**
 * \file    socket.h
 * WIZnet SOCKET API function definition
 *
 * For user application, WIZnet provides SOCKET API functions 
 * which are similar to Berkeley SOCKET API.\n
 *
 * \author MidnightCow
 * \date 04/07/2008
 * \version 1.1.1
 *
 * Modify the warning-block code in recv(). Refer to M_15052008.
 * ----------  -------  -----------  ----------------------------
 * Date        Version  Author       Description
 * ----------  -------  -----------  ----------------------------
 * 24/03/2008  1.0.0    MidnightCow  Release with W5300 launching
 * ----------  -------  -----------  ----------------------------
 * 15/05/2008  1.1.0    MidnightCow  Refer to M_15052008.
 *                                   Modify the warning code block in recv(). 
 * ----------  -------  -----------  ----------------------------  
 * 04/07/2008  1.1.1    MidnightCow  Refer to M_04072008.
 *                                   Modify the warning code block in recv(). 
 * ----------  -------  -----------  ----------------------------  
 * 08/08/2008  1.2.0    MidnightCow  Refer to M_08082008.
 *                                   Modify close(). 
 * ----------  -------  -----------  ----------------------------  
 */

#include "W5300_Conf.h"
#include "w5300.h"

#define     MAX_RECSIZE     2048*4
#define		W5300BUFSIZE	MAX_RECSIZE*4	

extern uint8	Rx_Buffer[W5300BUFSIZE];
extern uint8    Tx_Buffer[W5300BUFSIZE];

/**********************************
 * define function of SOCKET APIs * 
 **********************************/
void W5300_Socket_Init(SOCKET s, uint8 protocol, uint16 port, uint16 flag);
void W5300_Socket_Close(SOCKET s);                                                           // Close socket
uint8 W5300_Socket_Connect(SOCKET s, uint8 * addr, uint16 port);
void W5300_Socket_Disconnect(SOCKET s); 
uint8 W5300_Socket_Listen(SOCKET s);	    
uint32 S_tx_process(SOCKET s, uint8 * buf, uint32 len);
uint32 W5300_Socket_Recv(SOCKET s, uint8 * buf, uint32 len);
uint32 W5300_Socket_Sendto(SOCKET s, uint8 * buf, uint32 len, uint8 * addr, uint16 port);
uint32 W5300_Socket_Recvfrom(SOCKET s, uint8 * buf, uint32 len, uint8 * addr, uint16  *port); 
void W5300_loopback_tcps(SOCKET s, uint16 port, uint8* buf,uint16 mode);
uint8 W5300_State_management(SOCKET s, uint16 port, uint16 mode,uint8 *JG_IP,uint16 JG_DPort);
void W5300_loopback_udp(SOCKET s, uint16 port, uint8* buf, uint16 mode);
void W5300_Udp_socket(SOCKET s, uint8 protocol, uint16 port, uint16 flag);
extern uint32 S_rx_process(SOCKET s);
#endif
/* _SOCKET_H_ */


