//https://perso.uclouvain.be/fstandae/source_codes/hash_atmel/source/SHA256.asm
//some things adjusted
//.EQU    DATA_NUM_BYTE = 16

extern "C" void INIT();
extern "C" void UPDATE();
extern "C" void FINAL();

void sha_init() {
  asm("CALL INIT");
}

void sha_update_16(byte*) { //pointer to 16byte val
  asm("CALL UPDATE");
}

void sha_final_32(byte*) { //pointer to 32byte loc to write
  asm("ldi  r24,32");
  asm("ldi  r25,0");
  asm("CALL FINAL");
  
}
