
 //********* MODBUS STRUCTURES AND CONSTANTS ***************//
 //Initialization parameters
#CONST
  SERIAL:=1; //serial port to USE  0 for default port , 1 for serial port 1 (Use _1_ when SB2 and SB3 on MOTG-RS485 are soldered to RX/TX respectively)
  SLAVE_ADDRESS:= 0x10; // it could be set here or in each telegram
  MOD_TIMEOUT := 3000; //  timeout read/write modbus in ms
  MOD_POOL_T := 350; // pooling time period modbus in ms
  MOD_BAUDRATE := 1920; // 9600/10 this is the value required by diablo

#END


//Estructure modbus_t telegram
#CONST
     u8id
     u8fct
     u16RegAdd
     u16CoilsNo
     au16reg  //this is a pointer to the array that holds Modbus data
     size_modbus_t // size of structure
#END

//enum  exceptions
#CONST
    RESPONSE_SIZE := 6;
    EXCEPTION_SIZE := 3;
    CHECKSUM_SIZE := 2;
#END

 // FALSE TRUE enum
#CONST
    FALSE
    TRUE
#END

 //enum for telegram frame positions
#CONST
    ID := 0; //!< ID field
    FUNC :=1; //!< Function code position
    ADD_HI :=2; //!< Address high byte
    ADD_LO :=3; //!< Address low byte
    NB_HI :=4; //!< Number of coils or registers high byte
    NB_LO :=5; //!< Number of coils or registers low byte
    BYTE_CNT :=6; //!< byte counter
    SIZE_TELEGRAM:=7;
#END

//enum MB_FC, Modbus function code summary
#CONST
    MB_FC_NONE                     := 0;   /*!< null operator */
    MB_FC_READ_COILS               := 1;    /*!< FCT=1 -> read coils or digital outputs */
    MB_FC_READ_DISCRETE_INPUT      := 2;    /*!< FCT=2 -> read digital inputs */
    MB_FC_READ_REGISTERS           := 3;    /*!< FCT=3 -> read registers or analog outputs */
    MB_FC_READ_INPUT_REGISTER      := 4;    /*!< FCT=4 -> read analog inputs */
    MB_FC_WRITE_COIL               := 5;    /*!< FCT=5 -> write single coil or output */
    MB_FC_WRITE_REGISTER           := 6;    /*!< FCT=6 -> write single register */
    // the payload of these two cmd can be as long as 255 bytes(259B in total)
    MB_FC_WRITE_MULTIPLE_COILS     := 15;    /*!< FCT=15 -> write multiple coils or outputs */
    MB_FC_WRITE_MULTIPLE_REGISTERS := 16;    /*!< FCT=16 -> write multiple registers */
#END

 //enum COM_STATES
 #CONST
    COM_IDLE                     := 0;
    COM_WAITING                  := 1;
 #END

//enum ERR_LIST
#CONST
    ERR_NOT_MASTER                := -1;
    ERR_POLLING                   := -2;
    ERR_BUFF_OVERFLOW             := -3;
    ERR_BAD_CRC                   := -4;
    ERR_EXCEPTION                 := -5;
#END

//enum  errors
#CONST
    NO_REPLY := 255;
    EXC_FUNC_CODE := 1;
    EXC_ADDR_RANGE := 2;
    EXC_REGS_QUANT := 3;
    EXC_EXECUTE := 4;
#END

//Fucntion codes supported
var   fctsupported[8]:=[MB_FC_READ_COILS,MB_FC_READ_DISCRETE_INPUT,MB_FC_READ_REGISTERS,MB_FC_READ_INPUT_REGISTER,MB_FC_WRITE_COIL,MB_FC_WRITE_REGISTER,MB_FC_WRITE_MULTIPLE_COILS,MB_FC_WRITE_MULTIPLE_REGISTERS];

// Modbus extra parameters
#CONST
T35 := 5;  //delay between symbols or end of transmission
MAX_BUFFER := 64;       //!< maximum size for the communication buffer in bytes
#END

//base address and ofssets
#CONST
    BASE_RHOLD:=50;
    BASE_COILS:=0;
#END

//holding registers alias
#CONST
T_1A_RAW
T_2A_RAW
T_3A_RAW
T_4A_RAW
T_1B_RAW
T_2B_RAW
T_3B_RAW
T_4B_RAW
T_RES_1
T_RES_2
T_RES_3
T_RES_4
T_RES_5
T_RES_6
#END

// coils offset
#CONST
T_1A_OK
T_2A_OK
T_3A_OK
T_4A_OK
T_1B_OK
T_2B_OK
T_3B_OK
T_4B_OK

#END

// coils mask
#CONST
M_T_1A_OK := (0x0001)<< T_1A_OK ;
M_T_2A_OK := (0x0001)<< T_2A_OK ;
M_T_3A_OK := (0x0001)<< T_3A_OK ;
M_T_4A_OK := (0x0001)<< T_4A_OK ;
M_T_1B_OK := (0x0001)<< T_1B_OK ;
M_T_2B_OK := (0x0001)<< T_2B_OK ;
M_T_3B_OK := (0x0001)<< T_3B_OK ;
M_T_4B_OK := (0x0001)<< T_3B_OK ;
#END


//Modbus variables

var bufferRX[64];  //buffer RX modbus
var bufferTX[64];  //buffer TX modbus
var au16data[64];  // Modbus DATA holding registers for reading/writing
var au16Coils[4]; // data for writing coils


 var Modu8id; //!< 0=master, 1..247=slave number
 var Modu8serno; //!< serial port: 0-Serial, 1..3-Serial1..Serial3; 4: use software serial
 var Modu8txenpin; //!< flow control pin: 0=USB or RS-232 mode, >0=RS-485 mode
 var Modu8state;
 var Modu8lastError;
 var Modau8Buffer[MAX_BUFFER];
 var Modu8BufferSize;
 var Modu8lastRec;
 var *Modau16regs;  //pointer
 var Modu16InCnt, Modu16OutCnt, Modu16errCnt;
 var Modu16timeOut;
 var Modu32time[2], Modu32timeOut[2];
 var Modu8regsize;

 // Global variables screens
 var modVar, CurrentValue, NewValue;

 //keyboard variables
var i, status, n, x,y;

func ModbusTXEndEvent()
    pin_LO(PA6);
endfunc

/**
 * @brief
 * *** Only Modbus Master ***
 * Initialize serial port and communication buffers
 *
 * @param pin used for driving RS485 transceiver, 0 for RS232 point to point
 * @ingroup init
*/

func ModbusInit(var pin)
       //serial port initialization
  #IF  (SERIAL == 1)
    COM1_RX_pin(PA2);
    COM1_TX_pin(PA3);
    com1_Init(bufferRX,MAX_BUFFER,0);
    com1_TXbuffer(bufferTX, MAX_BUFFER,pin);
    com1_TXemptyEvent(ModbusTXEndEvent);
    com_SetBaud(COM1, MOD_BAUDRATE);
  #ELSE

    com_Init(bufferRX,MAX_BUFFER,0);
    com_TXbuffer(bufferTX, MAX_BUFFER,pin);
    com_SetBaud(COM0, MOD_BAUDRATE);

  #ENDIF

    pin_Set(PIN_OUT, PA6);            // set pin to required mode(OUTPUT, PA6);

    Modu8lastRec:=0;
    Modu8BufferSize:=0;
    Modu16InCnt := 0;
    Modu16OutCnt := 0;
    Modu16errCnt := 0;
    Modu8id := 0;
    Modu16timeOut:= MOD_TIMEOUT;  // watch dog time out communication for Modbus
    Modu8state:= COM_IDLE; // initial state of communication

endfunc


/**
 * @brief
 * *** Only Modbus Master ***
 * Generate a query to an slave with a modbus_t telegram structure
 * The Master must be in COM_IDLE mode. After it, its state would be COM_WAITING.
 * This method has to be called only in loop() section.
 *
 * @see modbus_t
 * @param modbus_t  modbus telegram structure (id, fct, ...)
 * @ingroup loop
*/

func ModQuery( var telegram )


    var u8regsno, u8bytesno, p;
    var i;

    if (Modu8id!=0)

      return (-2);
    endif
    if (Modu8state != COM_IDLE)
      return (-1);
    endif
    if ((telegram[u8id]==0) || (telegram[u8id]>247))
       return (-3);
    endif

    Modau16regs := telegram[au16reg];  // get the pointer to the data used in MODBUS

    p:=str_Ptr(Modau8Buffer); //obtiene el puntero del buffer de salida

    str_PutByte(p+ID, LObyte(telegram[u8id]));
    str_PutByte(p+FUNC, LObyte(telegram[u8fct]));
    str_PutByte(p+ADD_HI, HIbyte(telegram[u16RegAdd]));
    str_PutByte(p+ADD_LO, LObyte(telegram[u16RegAdd]));


    switch ( telegram[u8fct])
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUT:
        case MB_FC_READ_REGISTERS:
        case MB_FC_READ_INPUT_REGISTER:

             str_PutByte(p+NB_HI, HIbyte(telegram[u16CoilsNo]));
             str_PutByte(p+NB_LO, LObyte(telegram[u16CoilsNo]));

             Modu8BufferSize := 6;
             break;

        case MB_FC_WRITE_COIL:

            str_PutByte(p+NB_HI, ((Modau16regs[0]>0) ? 0xff : 0));
            str_PutByte(p+NB_LO, 0);

            Modu8BufferSize := 6;
            break;
        case MB_FC_WRITE_REGISTER:
            str_PutByte(p+NB_HI,HIbyte(Modau16regs[0]));
            str_PutByte(p+NB_LO,LObyte(Modau16regs[0]));

            Modu8BufferSize := 6;
            break;

         case MB_FC_WRITE_MULTIPLE_COILS: // TODO: implement "sending coils"
            u8regsno := telegram[u16CoilsNo] / 16;
            u8bytesno := u8regsno * 2;
            if ((telegram[u16CoilsNo] % 16) != 0)

                u8bytesno++;
                u8regsno++;
            endif

            str_PutByte(p+ NB_HI, HIbyte(telegram[u16CoilsNo]));
            str_PutByte(p+ NB_LO ,LObyte( telegram[u16CoilsNo]));
            str_PutByte(p+BYTE_CNT, u8bytesno);
            Modu8BufferSize := 7;

            for ( i := 0; i < u8bytesno; i++)

                if(i%2)

                   str_PutByte(p+Modu8BufferSize, LObyte( Modau16regs[ i/2 ] ));

                else

                    str_PutByte(p+Modu8BufferSize, HIbyte( Modau16regs[ i/2 ] ));
               endif
                Modu8BufferSize++;
            next
            break;

            case MB_FC_WRITE_MULTIPLE_REGISTERS:
            str_PutByte(p+ NB_HI , HIbyte(telegram[u16CoilsNo] ));
            str_PutByte(p+NB_LO , LObyte( telegram[u16CoilsNo]) );
            str_PutByte(p+ BYTE_CNT ,  telegram[u16CoilsNo]* 2 );
            Modu8BufferSize := 7;

            for ( i:=0; i< telegram[u16CoilsNo]; i++)
                str_PutByte(p+Modu8BufferSize ,HIbyte( Modau16regs[ i ] ));
                Modu8BufferSize++;
                str_PutByte(p+Modu8BufferSize ,LObyte( Modau16regs[ i ] ));
                Modu8BufferSize++;
            next
            break;

    endswitch

    ModSendTxBuffer();

    Modu8state := COM_WAITING;

    return 0;
endfunc

 //function to send the modbus stream
func ModSendTxBuffer()
  var i := 0;
  var *p;

  pin_HI(PA6);                    // set pin to logic '1'(OUTPUT, PA6);
  p:=str_Ptr(Modau8Buffer);

  var crc ;
  crc := crc_MODBUS(str_Ptr(Modau8Buffer), Modu8BufferSize);
  str_PutByte(p+Modu8BufferSize, (crc&0xff));
  Modu8BufferSize++;
  str_PutByte(p+Modu8BufferSize,(crc>>8)&0xff);
  Modu8BufferSize++;

  #IF  (SERIAL == 1)
     //com1_Init(bufferRX,MAX_BUFFER,0);
     com1_TXblock(str_Ptr(Modau8Buffer), Modu8BufferSize);

  #ELSE
     //com_Init(bufferRX,MAX_BUFFER,0);
     com_TXblock(str_Ptr(Modau8Buffer), Modu8BufferSize);
  #ENDIF
  Modu8BufferSize := 0;
  // set time-out for master

  sys_SetTimer(TIMER1, Modu16timeOut);

  // increase message counter
  Modu16OutCnt++;

endfunc



 /**
 * @brief *** Only for Modbus Master ***
 * This method checks if there is any incoming answer if pending.
 * If there is no answer, it would change Master state to COM_IDLE.
 * This method must be called only at loop section.
 * Avoid any delay() function.
 *
 * Any incoming data would be redirected to au16regs pointer,
 * as defined in its modbus_t query telegram.
 *
 * @params    nothing
 * @return errors counter
 * @ingroup loop
 */

func ModPoll()
   var u8current;
   //get the number of received bytes

#IF  (SERIAL == 1)
    u8current:=com1_Count();

#ELSE
     u8current:=com_Count();

#ENDIF

  txt_FontID(FONT_2);
  txt_FGcolour(YELLOW) ;
  txt_BGcolour(RED) ;
  txt_MoveCursor(0,0);

   if(sys_GetTimer(TIMER1)==0) //  timeout ocurred

        print("COM:Timeout");
        Modu8state := COM_IDLE;
        Modu8lastError := NO_REPLY;
        Modu16errCnt++;
        return 0;
   endif
   if (u8current == 0)
   //print("No bytes");
   return 0; // no bytes received
   endif
   // check T35 after frame end or still no frame end
    if (u8current != Modu8lastRec)
        Modu8lastRec := u8current;
        sys_SetTimer(TIMER2,T35);
        return 0;
    endif

    if(sys_GetTimer(TIMER2)>0)
         return 0;
    endif

      // transfer Serial buffer frame to auBuffer
    //print(Modu8lastRec);
    Modu8lastRec := 0;
    var i8state;
    i8state:= ModGetRxBuffer();

    //print(i8state);
    if (i8state < 6)  ///7 was incorrect for functions 1 and 2 the smallest frame could be 6 bytes long
         //print("COM:Error ");
         //print(i8state);
        Modu8state := COM_IDLE;
        Modu16errCnt++;
        return i8state;
    endif

     // validate message: id, CRC, FCT, exception
    var u8exception;
    u8exception :=  ModValidateAnswer();


    if (u8exception != 0)
        print("COM:Excep.");
        Modu8state := COM_IDLE;
        return u8exception;
    endif

    txt_FGcolour(LIME) ;
    txt_BGcolour(BLACK) ;
    print("COM:ok      ");

    var *p;
    p:=str_Ptr(Modau8Buffer);
     // process answer
    switch( str_GetByte(p+FUNC) )

        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUT:
            // call get_FC1 to transfer the incoming message to au16regs buffer
            ModGet_FC1( );
            break;
        case MB_FC_READ_INPUT_REGISTER:
        case MB_FC_READ_REGISTERS :
            // call get_FC3 to transfer the incoming message to au16regs buffer
            ModGet_FC3( );
            break;
        case MB_FC_WRITE_COIL:
        case MB_FC_WRITE_REGISTER :
        case MB_FC_WRITE_MULTIPLE_COILS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS :
            // nothing to do
            break;
        default:
        break;
    endswitch
    Modu8state := COM_IDLE;
    return Modu8BufferSize;


endfunc

func ModGet_FC1()
    var uint8_t ,u8byte, i, maxI;
    var *p;
    u8byte := 3;
     p:=str_Ptr(Modau8Buffer);

    for (i:=0; i< str_GetByte(p+2); i++)

        if(i%2)

            Modau16regs[i/2]:=    str_GetByte(p+i+u8byte)<<8 |  LObyte(Modau16regs[i/2]) ;

        else

            Modau16regs[i/2]:=   HIbyte(Modau16regs[i/2])<<8 |  str_GetByte(p+i+u8byte) ;
        endif

     next
endfunc

func ModGet_FC3()
    var u8byte, i,p;
    u8byte := 3;

    p:=str_Ptr(Modau8Buffer);

    for (i:=0; i< str_GetByte(p+2) /2; i++)

        Modau16regs[ i ] := ByteSwap(str_GetWord(p+ u8byte));
        u8byte := u8byte+2;
    next
endfunc


func ModValidateAnswer()
    var u16MsgCRC;
    var p,crc;
    p:=str_Ptr(Modau8Buffer);
    u16MsgCRC :=str_GetWord(p+Modu8BufferSize-2);    // get crc received

    if( crc_MODBUS(p, Modu8BufferSize-2 ) !=  u16MsgCRC)   //compare with calculated crc
        Modu16errCnt ++;
        //print(Modu8BufferSize);
       // print("\n error crc");
        return NO_REPLY;
    endif
     // check exception
     if ((str_GetByte(p + FUNC) & 0x80) != 0)
        // print("err exception");
        Modu16errCnt++;
        return ERR_EXCEPTION;
    endif

     // check fct code
    var isSupported := FALSE;
    var  i;
    for ( i:= 0; i< sizeof( fctsupported ); i++)

        if (fctsupported[i] == str_GetByte(p + FUNC))

            isSupported := 1;
            break;
        endif
     next

     if (!isSupported)
        // print("err f code not supported");
        Modu16errCnt ++;
        return EXC_FUNC_CODE;
    endif

    return 0; // OK, no exception code thrown


endfunc


func ModGetRxBuffer()
    var  bBuffOverflow := FALSE;
    var *p;
    Modu8BufferSize:=0;
    p:= str_Ptr(Modau8Buffer);
#IF (SERIAL==1)
    while(com1_Count())
            str_PutByte(p+Modu8BufferSize,serin1());
#ELSE
    while(com_Count())
            str_PutByte(p+Modu8BufferSize,serin());
#ENDIF
            Modu8BufferSize++;
            if (Modu8BufferSize >= MAX_BUFFER)
                bBuffOverflow := TRUE;
            endif
    wend

    Modu16InCnt++;
    if (bBuffOverflow)
         Modu16errCnt++;
         return ERR_BUFF_OVERFLOW;
    endif
    return Modu8BufferSize;

endfunc

/**
 * @brief
 * *** Only for Modbus Slave ***
 * This method checks if there is any incoming query
 * Afterwards, it would shoot a validation routine plus a register query
 * Avoid any delay() function !!!!
 * After a successful frame between the Master and the Slave, the time-out timer is reset.
 *
 * @param *regs  register table for communication exchange
 * @param u8size  size of the register table
 * @return 0 if no query, 1..4 if communication error, >4 if correct query processed
 * @ingroup loop
 */
func ModbusClientPoll( )
//    au16regs = regs;
    //u8regsize = u8size;

   var u8current;
   //get the number of received bytes

#IF  (SERIAL == 1)
    u8current:=com1_Count();

#ELSE
     u8current:=com_Count();

#ENDIF

//  txt_FontID(FONT_2);
//  txt_FGcolour(YELLOW) ;
//  txt_BGcolour(RED) ;
//  txt_MoveCursor(0,0);

   if (u8current == 0)
       //print("No bytes");
        return 0; // no bytes received
   endif

   // check T35 after frame end or still no frame end
    if (u8current != Modu8lastRec)
        Modu8lastRec := u8current;
        sys_SetTimer(TIMER2, T35);
        return 0;
    endif

    if(sys_GetTimer(TIMER2)>0)
         return 0;
    endif

      // transfer Serial buffer frame to auBuffer
    //print(Modu8lastRec);
    Modu8lastRec := 0;
    var i8state;
    i8state:= ModGetRxBuffer();

    //print(i8state);
    //if (i8state < 6)  ///7 was incorrect for functions 1 and 2 the smallest frame could be 6 bytes long
    if (i8state < 7)
         txt_MoveCursor(0,0);
         txt_FontID(FONT_2);
         print("COM:Error ");
         print(i8state);
        //Modu8state := COM_IDLE;
        Modu16errCnt++;
        return i8state;
    endif

    var *p;
    p:=str_Ptr(Modau8Buffer);

    // check slave id
    if (str_GetByte(p + ID) != SLAVE_ADDRESS) return 0;

    // validate message: id, CRC, FCT, exception
    var u8exception;
    u8exception :=  ModValidateAnswer();

    if (u8exception != 0)
        txt_MoveCursor(0,0);
        txt_FontID(FONT_2);
        print("COM:Excep.");
        Modu8state := COM_IDLE;
        return u8exception;
    endif

    //u32timeOut = millis();
    //u8lastError = 0;

    // process message
    switch( str_GetByte(p+FUNC) )
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUT:
            return process_FC1( );
            break;
        case MB_FC_READ_INPUT_REGISTER:
        case MB_FC_READ_REGISTERS :
            return process_FC3( );
            break;
        case MB_FC_WRITE_COIL:
            return process_FC5( );
            break;
        case MB_FC_WRITE_REGISTER :
            return process_FC6( );
            break;
        case MB_FC_WRITE_MULTIPLE_COILS:
            return process_FC15( );
            break;
        case MB_FC_WRITE_MULTIPLE_REGISTERS :
            return process_FC16( );
            break;
        default:
        break;
    endswitch
    return i8state;
endfunc

func process_FC1()
    //MB_FC_READ_COILS               = 1,    /*!< FCT=1 -> read coils or digital outputs */
    //MB_FC_READ_DISCRETE_INPUT      = 2,    /*!< FCT=2 -> read digital inputs */
    return 0;
endfunc

func process_FC3()
    //MB_FC_READ_REGISTERS           = 3,    /*!< FCT=3 -> read registers or analog outputs */
    //MB_FC_READ_INPUT_REGISTER      = 4,    /*!< FCT=4 -> read analog inputs */
    var *p;
    p:= str_Ptr(Modau8Buffer);

    var u8StartAdd := 0;
    u8StartAdd := ByteSwap(str_GetWord(p + ADD_HI)); //word( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ] );

    var u8regsno := 0;
    u8regsno := ByteSwap(str_GetWord(p + NB_HI));// word( au8Buffer[ NB_HI ], au8Buffer[ NB_LO ] );

    var u8CopyBufferSize;

    p[ 2 ] := u8regsno * 2;
    Modu8BufferSize := 3;

    for (i := u8StartAdd; i < u8StartAdd + u8regsno; i++)
        p[ Modu8BufferSize ] := HIbyte(au16data[i]);
        Modu8BufferSize++;
        p[ Modu8BufferSize ] := LObyte(au16data[i]);
        Modu8BufferSize++;
    next
    u8CopyBufferSize := Modu8BufferSize +2;
    ModSendTxBuffer();
    return u8CopyBufferSize;
endfunc

func process_FC5()
    //MB_FC_WRITE_COIL               = 5,    /*!< FCT=5 -> write single coil or output */
    return 0;
endfunc

func process_FC6()
    //MB_FC_WRITE_REGISTER           = 6,    /*!< FCT=6 -> write single register */
    var *p;
    p:= str_Ptr(Modau8Buffer);

    var u8add := 0;
    u8add := ByteSwap(str_GetWord(p + ADD_HI)); //word( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ] );

    var u8CopyBufferSize;

    var u16val := 0;
    u16val := ByteSwap(str_GetWord(p + NB_HI));// word( au8Buffer[ NB_HI ], au8Buffer[ NB_LO ] );

    au16data[ u8add ] := u16val;

    // keep the same header
    Modu8BufferSize := RESPONSE_SIZE;

    u8CopyBufferSize := Modu8BufferSize +2;
    ModSendTxBuffer();

    return u8CopyBufferSize;
endfunc

func process_FC15()
    //MB_FC_WRITE_MULTIPLE_COILS     = 15,    /*!< FCT=15 -> write multiple coils or outputs */
    return 0  ;
endfunc

func process_FC16()
    //MB_FC_WRITE_MULTIPLE_REGISTERS = 16    /*!< FCT=16 -> write multiple registers */
    var *p;
    p:= str_Ptr(Modau8Buffer);

    var u8StartAdd := 0;
    u8StartAdd := ByteSwap(str_GetWord(p + ADD_HI));

    var u8regsno := 0;
    u8regsno := ByteSwap(str_GetWord(p + NB_HI));

    var u8CopyBufferSize;
    var temp;

    // build header
    p[ NB_HI ] := 0;
    p[ NB_LO ] := u8regsno;
    Modu8BufferSize := RESPONSE_SIZE;

    // write registers
    for (i := 0; i < u8regsno; i++)
        temp := ByteSwap(str_GetWord(p + (BYTE_CNT+ 1) + i * 2));

        au16data[ u8StartAdd + i ] := temp;
    next
    u8CopyBufferSize := Modu8BufferSize +2;
    ModSendTxBuffer();

    return u8CopyBufferSize;
endfunc
