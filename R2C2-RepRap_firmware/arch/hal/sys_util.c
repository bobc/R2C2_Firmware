/* ========================================================================== */
/*                                                                            */
/*   Filename.c                                                               */
/*   (c) 2001 Author                                                          */
/*                                                                            */
/*   Description                                                              */
/*                                                                            */
/* ========================================================================== */


void startup_delay(void)
{
  for (volatile unsigned long i = 0; i < 500000; i++) { ; }
}