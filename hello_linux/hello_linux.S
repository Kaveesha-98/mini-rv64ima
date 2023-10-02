
hello_linux.gdb:     file format elf64-littleriscv


Disassembly of section .text:

0000000000000000 <_start-0x4>:
       0:	0000                	.2byte	0x0
	...

0000000000000004 <_start>:
       4:	02c000ef          	jal	ra,30 <.Lload_gp>
       8:	00050793          	mv	a5,a0
       c:	00005517          	auipc	a0,0x5
      10:	5b850513          	addi	a0,a0,1464 # 55c4 <main>
      14:	00013583          	ld	a1,0(sp)
      18:	00810613          	addi	a2,sp,8
      1c:	ff017113          	andi	sp,sp,-16
      20:	00000693          	li	a3,0
      24:	00000713          	li	a4,0
      28:	00010813          	mv	a6,sp
      2c:	1510106f          	j	197c <__uClibc_main>

0000000000000030 <.Lload_gp>:
      30:	00006197          	auipc	gp,0x6
      34:	eb018193          	addi	gp,gp,-336 # 5ee0 <__global_pointer$>
      38:	00008067          	ret

000000000000003c <deregister_tm_clones>:
      3c:	00007517          	auipc	a0,0x7
      40:	aac50513          	addi	a0,a0,-1364 # 6ae8 <__TMC_END__>
      44:	00007797          	auipc	a5,0x7
      48:	aa478793          	addi	a5,a5,-1372 # 6ae8 <__TMC_END__>
      4c:	00a78a63          	beq	a5,a0,60 <.L1>
      50:	00005797          	auipc	a5,0x5
      54:	6187b783          	ld	a5,1560(a5) # 5668 <.LLST14+0x1c>
      58:	00078463          	beqz	a5,60 <.L1>
      5c:	00078067          	jr	a5

0000000000000060 <.L1>:
      60:	00008067          	ret

0000000000000064 <register_tm_clones>:
      64:	00007517          	auipc	a0,0x7
      68:	a8450513          	addi	a0,a0,-1404 # 6ae8 <__TMC_END__>
      6c:	00007597          	auipc	a1,0x7
      70:	a7c58593          	addi	a1,a1,-1412 # 6ae8 <__TMC_END__>
      74:	40a585b3          	sub	a1,a1,a0
      78:	4035d593          	srai	a1,a1,0x3
      7c:	00200793          	li	a5,2
      80:	02f5c5b3          	div	a1,a1,a5
      84:	00058a63          	beqz	a1,98 <.L6>
      88:	00005797          	auipc	a5,0x5
      8c:	5c87b783          	ld	a5,1480(a5) # 5650 <.LLST14+0x4>
      90:	00078463          	beqz	a5,98 <.L6>
      94:	00078067          	jr	a5

0000000000000098 <.L6>:
      98:	00008067          	ret

000000000000009c <__do_global_dtors_aux>:
      9c:	ff010113          	addi	sp,sp,-16
      a0:	00813023          	sd	s0,0(sp)
      a4:	00007417          	auipc	s0,0x7
      a8:	a4c40413          	addi	s0,s0,-1460 # 6af0 <completed.1>
      ac:	00044783          	lbu	a5,0(s0)
      b0:	00113423          	sd	ra,8(sp)
      b4:	02079463          	bnez	a5,dc <.L14>
      b8:	f85ff0ef          	jal	ra,3c <deregister_tm_clones>
      bc:	00005797          	auipc	a5,0x5
      c0:	58c7b783          	ld	a5,1420(a5) # 5648 <_GLOBAL_OFFSET_TABLE_+0x38>
      c4:	00078863          	beqz	a5,d4 <.L16>
      c8:	00007517          	auipc	a0,0x7
      cc:	a2050513          	addi	a0,a0,-1504 # 6ae8 <__TMC_END__>
      d0:	000780e7          	jalr	a5

00000000000000d4 <.L16>:
      d4:	00100793          	li	a5,1
      d8:	00f40023          	sb	a5,0(s0)

00000000000000dc <.L14>:
      dc:	00813083          	ld	ra,8(sp)
      e0:	00013403          	ld	s0,0(sp)
      e4:	01010113          	addi	sp,sp,16
      e8:	00008067          	ret

00000000000000ec <frame_dummy>:
      ec:	00005797          	auipc	a5,0x5
      f0:	5b47b783          	ld	a5,1460(a5) # 56a0 <.LLST14+0x54>
      f4:	02078463          	beqz	a5,11c <.L27>
      f8:	ff010113          	addi	sp,sp,-16
      fc:	00113423          	sd	ra,8(sp)
     100:	00007597          	auipc	a1,0x7
     104:	9f858593          	addi	a1,a1,-1544 # 6af8 <object.0>
     108:	00007517          	auipc	a0,0x7
     10c:	9e050513          	addi	a0,a0,-1568 # 6ae8 <__TMC_END__>
     110:	000780e7          	jalr	a5
     114:	00813083          	ld	ra,8(sp)
     118:	01010113          	addi	sp,sp,16

000000000000011c <.L27>:
     11c:	f49ff06f          	j	64 <register_tm_clones>

0000000000000120 <__GI_printf>:
     120:	fa010113          	addi	sp,sp,-96
     124:	02b13423          	sd	a1,40(sp)
     128:	02c13823          	sd	a2,48(sp)
     12c:	04f13423          	sd	a5,72(sp)
     130:	02d13c23          	sd	a3,56(sp)
     134:	04e13023          	sd	a4,64(sp)
     138:	05013823          	sd	a6,80(sp)
     13c:	05113c23          	sd	a7,88(sp)
     140:	00005797          	auipc	a5,0x5
     144:	4d87b783          	ld	a5,1240(a5) # 5618 <_GLOBAL_OFFSET_TABLE_+0x8>
     148:	00050593          	mv	a1,a0
     14c:	0007b503          	ld	a0,0(a5)
     150:	02810613          	addi	a2,sp,40
     154:	00113c23          	sd	ra,24(sp)
     158:	00c13423          	sd	a2,8(sp)
     15c:	134000ef          	jal	ra,290 <__GI_vfprintf>
     160:	01813083          	ld	ra,24(sp)
     164:	06010113          	addi	sp,sp,96
     168:	00008067          	ret

000000000000016c <_stdio_term>:
     16c:	fe010113          	addi	sp,sp,-32
     170:	00005797          	auipc	a5,0x5
     174:	4d07b783          	ld	a5,1232(a5) # 5640 <_GLOBAL_OFFSET_TABLE_+0x30>
     178:	00813823          	sd	s0,16(sp)
     17c:	0007b403          	ld	s0,0(a5)
     180:	00913423          	sd	s1,8(sp)
     184:	00113c23          	sd	ra,24(sp)
     188:	ffe00493          	li	s1,-2

000000000000018c <.L2>:
     18c:	00041c63          	bnez	s0,1a4 <.L5>
     190:	01813083          	ld	ra,24(sp)
     194:	01013403          	ld	s0,16(sp)
     198:	00813483          	ld	s1,8(sp)
     19c:	02010113          	addi	sp,sp,32
     1a0:	00008067          	ret

00000000000001a4 <.L5>:
     1a4:	00045783          	lhu	a5,0(s0)
     1a8:	0407f793          	andi	a5,a5,64
     1ac:	00078663          	beqz	a5,1b8 <.L3>
     1b0:	00040513          	mv	a0,s0
     1b4:	098000ef          	jal	ra,24c <__stdio_wcommit>

00000000000001b8 <.L3>:
     1b8:	00442783          	lw	a5,4(s0)
     1bc:	00979a63          	bne	a5,s1,1d0 <.L4>
     1c0:	06043783          	ld	a5,96(s0)
     1c4:	00078663          	beqz	a5,1d0 <.L4>
     1c8:	04043503          	ld	a0,64(s0)
     1cc:	000780e7          	jalr	a5

00000000000001d0 <.L4>:
     1d0:	03843403          	ld	s0,56(s0)
     1d4:	fb9ff06f          	j	18c <.L2>

00000000000001d8 <_stdio_init>:
     1d8:	fe010113          	addi	sp,sp,-32
     1dc:	00813823          	sd	s0,16(sp)
     1e0:	00000513          	li	a0,0
     1e4:	00005417          	auipc	s0,0x5
     1e8:	44443403          	ld	s0,1092(s0) # 5628 <_GLOBAL_OFFSET_TABLE_+0x18>
     1ec:	00913423          	sd	s1,8(sp)
     1f0:	00113c23          	sd	ra,24(sp)
     1f4:	00042483          	lw	s1,0(s0)
     1f8:	698010ef          	jal	ra,1890 <__GI_isatty>
     1fc:	00051c63          	bnez	a0,214 <.L14>
     200:	00006717          	auipc	a4,0x6
     204:	7b070713          	addi	a4,a4,1968 # 69b0 <_stdio_streams>
     208:	00075783          	lhu	a5,0(a4)
     20c:	1007c793          	xori	a5,a5,256
     210:	00f71023          	sh	a5,0(a4)

0000000000000214 <.L14>:
     214:	00100513          	li	a0,1
     218:	678010ef          	jal	ra,1890 <__GI_isatty>
     21c:	00051c63          	bnez	a0,234 <.L15>
     220:	00006717          	auipc	a4,0x6
     224:	79070713          	addi	a4,a4,1936 # 69b0 <_stdio_streams>
     228:	04075783          	lhu	a5,64(a4)
     22c:	1007c793          	xori	a5,a5,256
     230:	04f71023          	sh	a5,64(a4)

0000000000000234 <.L15>:
     234:	00942023          	sw	s1,0(s0)
     238:	01813083          	ld	ra,24(sp)
     23c:	01013403          	ld	s0,16(sp)
     240:	00813483          	ld	s1,8(sp)
     244:	02010113          	addi	sp,sp,32
     248:	00008067          	ret

000000000000024c <__stdio_wcommit>:
     24c:	00853583          	ld	a1,8(a0)
     250:	01853783          	ld	a5,24(a0)
     254:	ff010113          	addi	sp,sp,-16
     258:	00813023          	sd	s0,0(sp)
     25c:	00113423          	sd	ra,8(sp)
     260:	00050413          	mv	s0,a0
     264:	40b78633          	sub	a2,a5,a1
     268:	00b78663          	beq	a5,a1,274 <.L2>
     26c:	00b53c23          	sd	a1,24(a0)
     270:	101010ef          	jal	ra,1b70 <__stdio_WRITE>

0000000000000274 <.L2>:
     274:	01843503          	ld	a0,24(s0)
     278:	00843783          	ld	a5,8(s0)
     27c:	00813083          	ld	ra,8(sp)
     280:	00013403          	ld	s0,0(sp)
     284:	40f50533          	sub	a0,a0,a5
     288:	01010113          	addi	sp,sp,16
     28c:	00008067          	ret

0000000000000290 <__GI_vfprintf>:
     290:	00055783          	lhu	a5,0(a0)
     294:	fe010113          	addi	sp,sp,-32
     298:	00813823          	sd	s0,16(sp)
     29c:	00113c23          	sd	ra,24(sp)
     2a0:	0407f793          	andi	a5,a5,64
     2a4:	00050413          	mv	s0,a0
     2a8:	00078c63          	beqz	a5,2c0 <.L2>

00000000000002ac <.L3>:
     2ac:	00040513          	mv	a0,s0
     2b0:	01013403          	ld	s0,16(sp)
     2b4:	01813083          	ld	ra,24(sp)
     2b8:	02010113          	addi	sp,sp,32
     2bc:	1580006f          	j	414 <_vfprintf_internal>

00000000000002c0 <.L2>:
     2c0:	00c13423          	sd	a2,8(sp)
     2c4:	00b13023          	sd	a1,0(sp)
     2c8:	355010ef          	jal	ra,1e1c <__stdio_trans2w>
     2cc:	00013583          	ld	a1,0(sp)
     2d0:	00813603          	ld	a2,8(sp)
     2d4:	fc050ce3          	beqz	a0,2ac <.L3>
     2d8:	01813083          	ld	ra,24(sp)
     2dc:	01013403          	ld	s0,16(sp)
     2e0:	fff00513          	li	a0,-1
     2e4:	02010113          	addi	sp,sp,32
     2e8:	00008067          	ret

00000000000002ec <_charpad>:
     2ec:	fc010113          	addi	sp,sp,-64
     2f0:	02813823          	sd	s0,48(sp)
     2f4:	02913423          	sd	s1,40(sp)
     2f8:	03213023          	sd	s2,32(sp)
     2fc:	01313c23          	sd	s3,24(sp)
     300:	02113c23          	sd	ra,56(sp)
     304:	00050913          	mv	s2,a0
     308:	00060493          	mv	s1,a2
     30c:	00b10423          	sb	a1,8(sp)
     310:	00060413          	mv	s0,a2
     314:	00810993          	addi	s3,sp,8

0000000000000318 <.L2>:
     318:	00040e63          	beqz	s0,334 <.L3>
     31c:	00090613          	mv	a2,s2
     320:	00100593          	li	a1,1
     324:	00098513          	mv	a0,s3
     328:	1a5010ef          	jal	ra,1ccc <__stdio_fwrite>
     32c:	00100793          	li	a5,1
     330:	02f50263          	beq	a0,a5,354 <.L4>

0000000000000334 <.L3>:
     334:	40848533          	sub	a0,s1,s0
     338:	03813083          	ld	ra,56(sp)
     33c:	03013403          	ld	s0,48(sp)
     340:	02813483          	ld	s1,40(sp)
     344:	02013903          	ld	s2,32(sp)
     348:	01813983          	ld	s3,24(sp)
     34c:	04010113          	addi	sp,sp,64
     350:	00008067          	ret

0000000000000354 <.L4>:
     354:	fff40413          	addi	s0,s0,-1
     358:	fc1ff06f          	j	318 <.L2>

000000000000035c <_fp_out_narrow>:
     35c:	fc010113          	addi	sp,sp,-64
     360:	02913423          	sd	s1,40(sp)
     364:	01313c23          	sd	s3,24(sp)
     368:	01413823          	sd	s4,16(sp)
     36c:	02113c23          	sd	ra,56(sp)
     370:	02813823          	sd	s0,48(sp)
     374:	03213023          	sd	s2,32(sp)
     378:	01513423          	sd	s5,8(sp)
     37c:	0805f793          	andi	a5,a1,128
     380:	00050a13          	mv	s4,a0
     384:	00060493          	mv	s1,a2
     388:	00068993          	mv	s3,a3
     38c:	06078e63          	beqz	a5,408 <.L13>
     390:	00068513          	mv	a0,a3
     394:	00058a93          	mv	s5,a1
     398:	79d000ef          	jal	ra,1334 <__GI_strlen>
     39c:	0005091b          	sext.w	s2,a0
     3a0:	412484b3          	sub	s1,s1,s2
     3a4:	00000413          	li	s0,0
     3a8:	00905e63          	blez	s1,3c4 <.L10>
     3ac:	00048613          	mv	a2,s1
     3b0:	07faf593          	andi	a1,s5,127
     3b4:	000a0513          	mv	a0,s4
     3b8:	f35ff0ef          	jal	ra,2ec <_charpad>
     3bc:	00050413          	mv	s0,a0
     3c0:	02a49063          	bne	s1,a0,3e0 <.L9>

00000000000003c4 <.L10>:
     3c4:	00000513          	li	a0,0
     3c8:	01205a63          	blez	s2,3dc <.L12>
     3cc:	000a0613          	mv	a2,s4
     3d0:	00090593          	mv	a1,s2
     3d4:	00098513          	mv	a0,s3
     3d8:	0f5010ef          	jal	ra,1ccc <__stdio_fwrite>

00000000000003dc <.L12>:
     3dc:	00a40433          	add	s0,s0,a0

00000000000003e0 <.L9>:
     3e0:	03813083          	ld	ra,56(sp)
     3e4:	00040513          	mv	a0,s0
     3e8:	03013403          	ld	s0,48(sp)
     3ec:	02813483          	ld	s1,40(sp)
     3f0:	02013903          	ld	s2,32(sp)
     3f4:	01813983          	ld	s3,24(sp)
     3f8:	01013a03          	ld	s4,16(sp)
     3fc:	00813a83          	ld	s5,8(sp)
     400:	04010113          	addi	sp,sp,64
     404:	00008067          	ret

0000000000000408 <.L13>:
     408:	00060913          	mv	s2,a2
     40c:	00000413          	li	s0,0
     410:	fb5ff06f          	j	3c4 <.L10>

0000000000000414 <_vfprintf_internal>:
     414:	dd010113          	addi	sp,sp,-560
     418:	21313423          	sd	s3,520(sp)
     41c:	0d010993          	addi	s3,sp,208
     420:	21213823          	sd	s2,528(sp)
     424:	00050913          	mv	s2,a0
     428:	00098513          	mv	a0,s3
     42c:	22813023          	sd	s0,544(sp)
     430:	20913c23          	sd	s1,536(sp)
     434:	22113423          	sd	ra,552(sp)
     438:	21413023          	sd	s4,512(sp)
     43c:	1f513c23          	sd	s5,504(sp)
     440:	1f613823          	sd	s6,496(sp)
     444:	1f713423          	sd	s7,488(sp)
     448:	1f813023          	sd	s8,480(sp)
     44c:	1d913c23          	sd	s9,472(sp)
     450:	1da13823          	sd	s10,464(sp)
     454:	1db13423          	sd	s11,456(sp)
     458:	00058413          	mv	s0,a1
     45c:	00060493          	mv	s1,a2
     460:	58c000ef          	jal	ra,9ec <_ppfs_init>
     464:	06055463          	bgez	a0,4cc <.L18>
     468:	0d013403          	ld	s0,208(sp)
     46c:	fff00493          	li	s1,-1
     470:	00040513          	mv	a0,s0
     474:	6c1000ef          	jal	ra,1334 <__GI_strlen>
     478:	00050593          	mv	a1,a0
     47c:	00050863          	beqz	a0,48c <.L89>
     480:	00090613          	mv	a2,s2
     484:	00040513          	mv	a0,s0
     488:	045010ef          	jal	ra,1ccc <__stdio_fwrite>

000000000000048c <.L89>:
     48c:	22813083          	ld	ra,552(sp)
     490:	22013403          	ld	s0,544(sp)
     494:	21013903          	ld	s2,528(sp)
     498:	20813983          	ld	s3,520(sp)
     49c:	20013a03          	ld	s4,512(sp)
     4a0:	1f813a83          	ld	s5,504(sp)
     4a4:	1f013b03          	ld	s6,496(sp)
     4a8:	1e813b83          	ld	s7,488(sp)
     4ac:	1e013c03          	ld	s8,480(sp)
     4b0:	1d813c83          	ld	s9,472(sp)
     4b4:	1d013d03          	ld	s10,464(sp)
     4b8:	1c813d83          	ld	s11,456(sp)
     4bc:	00048513          	mv	a0,s1
     4c0:	21813483          	ld	s1,536(sp)
     4c4:	23010113          	addi	sp,sp,560
     4c8:	00008067          	ret

00000000000004cc <.L18>:
     4cc:	00048593          	mv	a1,s1
     4d0:	00098513          	mv	a0,s3
     4d4:	5fc000ef          	jal	ra,ad0 <_ppfs_prepargs>
     4d8:	00001a37          	lui	s4,0x1
     4dc:	00040513          	mv	a0,s0
     4e0:	00000493          	li	s1,0
     4e4:	13010a93          	addi	s5,sp,304
     4e8:	0d810b13          	addi	s6,sp,216
     4ec:	800a0d13          	addi	s10,s4,-2048 # 800 <.L49+0x8>

00000000000004f0 <.L20>:
     4f0:	00044783          	lbu	a5,0(s0)
     4f4:	44078263          	beqz	a5,938 <.L21>
     4f8:	02500713          	li	a4,37
     4fc:	00e79663          	bne	a5,a4,508 <.L22>
     500:	46a40263          	beq	s0,a0,964 <.L69>
     504:	4380006f          	j	93c <.L68>

0000000000000508 <.L22>:
     508:	00140413          	addi	s0,s0,1
     50c:	fe5ff06f          	j	4f0 <.L20>

0000000000000510 <.L29>:
     510:	00f6b023          	sd	a5,0(a3)
     514:	0015859b          	addiw	a1,a1,1
     518:	00868693          	addi	a3,a3,8
     51c:	01078793          	addi	a5,a5,16
     520:	4980006f          	j	9b8 <.L28>

0000000000000524 <.L30>:
     524:	00700693          	li	a3,7
     528:	28f6e663          	bltu	a3,a5,7b4 <.L32>
     52c:	fff7861b          	addiw	a2,a5,-1
     530:	00006697          	auipc	a3,0x6
     534:	3c868693          	addi	a3,a3,968 # 68f8 <spec_base.1>
     538:	00c686b3          	add	a3,a3,a2
     53c:	0006c603          	lbu	a2,0(a3)
     540:	00a00593          	li	a1,10
     544:	05700693          	li	a3,87
     548:	00060513          	mv	a0,a2
     54c:	02b61063          	bne	a2,a1,56c <.L33>
     550:	0e412583          	lw	a1,228(sp)
     554:	0205f813          	andi	a6,a1,32
     558:	00080463          	beqz	a6,560 <.L34>
     55c:	02c00693          	li	a3,44

0000000000000560 <.L34>:
     560:	0405f593          	andi	a1,a1,64
     564:	00058463          	beqz	a1,56c <.L33>
     568:	0806e693          	ori	a3,a3,128

000000000000056c <.L33>:
     56c:	00500593          	li	a1,5
     570:	10f5ee63          	bltu	a1,a5,68c <.L35>
     574:	00300593          	li	a1,3
     578:	12b78063          	beq	a5,a1,698 <.L74>
     57c:	00100593          	li	a1,1
     580:	00b00c93          	li	s9,11
     584:	00b79463          	bne	a5,a1,58c <.L36>
     588:	00600c93          	li	s9,6

000000000000058c <.L36>:
     58c:	0d812583          	lw	a1,216(sp)
     590:	02000893          	li	a7,32
     594:	0005d463          	bgez	a1,59c <.L37>
     598:	0e814883          	lbu	a7,232(sp)

000000000000059c <.L37>:
     59c:	00100593          	li	a1,1
     5a0:	40000513          	li	a0,1024
     5a4:	02b78063          	beq	a5,a1,5c4 <.L38>
     5a8:	00271713          	slli	a4,a4,0x2
     5ac:	1a070793          	addi	a5,a4,416
     5b0:	02010713          	addi	a4,sp,32
     5b4:	00e78733          	add	a4,a5,a4
     5b8:	f3872503          	lw	a0,-200(a4)
     5bc:	f00a0793          	addi	a5,s4,-256
     5c0:	00f57533          	and	a0,a0,a5

00000000000005c4 <.L38>:
     5c4:	02813583          	ld	a1,40(sp)
     5c8:	01113c23          	sd	a7,24(sp)
     5cc:	00d13823          	sd	a3,16(sp)
     5d0:	00c13423          	sd	a2,8(sp)
     5d4:	119010ef          	jal	ra,1eec <_load_inttype>
     5d8:	00813603          	ld	a2,8(sp)
     5dc:	01013683          	ld	a3,16(sp)
     5e0:	0cf10413          	addi	s0,sp,207
     5e4:	00050593          	mv	a1,a0
     5e8:	00040513          	mv	a0,s0
     5ec:	1c1010ef          	jal	ra,1fac <_uintmaxtostr>
     5f0:	0f412603          	lw	a2,244(sp)
     5f4:	00500793          	li	a5,5
     5f8:	01813883          	ld	a7,24(sp)
     5fc:	00050c13          	mv	s8,a0
     600:	02c7f263          	bgeu	a5,a2,624 <.L39>
     604:	00054583          	lbu	a1,0(a0)
     608:	02d00693          	li	a3,45
     60c:	0e412783          	lw	a5,228(sp)
     610:	08d59a63          	bne	a1,a3,6a4 <.L40>
     614:	0027e793          	ori	a5,a5,2
     618:	0ef12223          	sw	a5,228(sp)
     61c:	00150c13          	addi	s8,a0,1
     620:	00200c93          	li	s9,2

0000000000000624 <.L39>:
     624:	418407b3          	sub	a5,s0,s8
     628:	0d812403          	lw	s0,216(sp)
     62c:	00078b93          	mv	s7,a5
     630:	00045463          	bgez	s0,638 <.L41>
     634:	00100413          	li	s0,1

0000000000000638 <.L41>:
     638:	0e412583          	lw	a1,228(sp)
     63c:	000c4503          	lbu	a0,0(s8)
     640:	0105f593          	andi	a1,a1,16
     644:	00058e63          	beqz	a1,660 <.L42>
     648:	00200593          	li	a1,2
     64c:	06c5fc63          	bgeu	a1,a2,6c4 <.L80>
     650:	00300593          	li	a1,3
     654:	08b60863          	beq	a2,a1,6e4 <.L81>
     658:	00400593          	li	a1,4
     65c:	06b60863          	beq	a2,a1,6cc <.L44>

0000000000000660 <.L42>:
     660:	03000593          	li	a1,48
     664:	08b51c63          	bne	a0,a1,6fc <.L45>
     668:	00500593          	li	a1,5
     66c:	0195d463          	bge	a1,s9,674 <.L46>
     670:	00b00c93          	li	s9,11

0000000000000674 <.L46>:
     674:	00100593          	li	a1,1
     678:	06b60a63          	beq	a2,a1,6ec <.L83>
     67c:	08041063          	bnez	s0,6fc <.L45>
     680:	00000b93          	li	s7,0

0000000000000684 <.L84>:
     684:	00000413          	li	s0,0
     688:	07c0006f          	j	704 <.L47>

000000000000068c <.L35>:
     68c:	40a0063b          	negw	a2,a0
     690:	00b00c93          	li	s9,11
     694:	ef9ff06f          	j	58c <.L36>

0000000000000698 <.L74>:
     698:	00b00c93          	li	s9,11
     69c:	03700693          	li	a3,55
     6a0:	eedff06f          	j	58c <.L36>

00000000000006a4 <.L40>:
     6a4:	0027f693          	andi	a3,a5,2
     6a8:	00069a63          	bnez	a3,6bc <.L78>
     6ac:	0017f793          	andi	a5,a5,1
     6b0:	f6078ae3          	beqz	a5,624 <.L39>
     6b4:	00400c93          	li	s9,4
     6b8:	f6dff06f          	j	624 <.L39>

00000000000006bc <.L78>:
     6bc:	00000c93          	li	s9,0
     6c0:	f65ff06f          	j	624 <.L39>

00000000000006c4 <.L80>:
     6c4:	00600c93          	li	s9,6
     6c8:	f99ff06f          	j	660 <.L42>

00000000000006cc <.L44>:
     6cc:	f887eae3          	bltu	a5,s0,660 <.L42>
     6d0:	03000593          	li	a1,48
     6d4:	00100413          	li	s0,1
     6d8:	f8b504e3          	beq	a0,a1,660 <.L42>
     6dc:	0017841b          	addiw	s0,a5,1
     6e0:	f81ff06f          	j	660 <.L42>

00000000000006e4 <.L81>:
     6e4:	00900c93          	li	s9,9
     6e8:	f79ff06f          	j	660 <.L42>

00000000000006ec <.L83>:
     6ec:	00006c17          	auipc	s8,0x6
     6f0:	1fcc0c13          	addi	s8,s8,508 # 68e8 <.LC0>
     6f4:	00500b93          	li	s7,5
     6f8:	f8dff06f          	j	684 <.L84>

00000000000006fc <.L45>:
     6fc:	f887f4e3          	bgeu	a5,s0,684 <.L84>
     700:	40f4043b          	subw	s0,s0,a5

0000000000000704 <.L47>:
     704:	00b00793          	li	a5,11
     708:	01740633          	add	a2,s0,s7
     70c:	00fc8a63          	beq	s9,a5,720 <.L60>
     710:	00500793          	li	a5,5
     714:	0197a7b3          	slt	a5,a5,s9
     718:	00178793          	addi	a5,a5,1
     71c:	00f60633          	add	a2,a2,a5

0000000000000720 <.L60>:
     720:	0dc12583          	lw	a1,220(sp)
     724:	0006079b          	sext.w	a5,a2
     728:	00000d93          	li	s11,0
     72c:	00b67463          	bgeu	a2,a1,734 <.L62>
     730:	40f58dbb          	subw	s11,a1,a5

0000000000000734 <.L62>:
     734:	00f487bb          	addw	a5,s1,a5
     738:	01b784bb          	addw	s1,a5,s11
     73c:	03000793          	li	a5,48
     740:	00f89663          	bne	a7,a5,74c <.L63>
     744:	01b4043b          	addw	s0,s0,s11
     748:	00000d93          	li	s11,0

000000000000074c <.L63>:
     74c:	0e412783          	lw	a5,228(sp)
     750:	0087f793          	andi	a5,a5,8
     754:	00079e63          	bnez	a5,770 <.L64>
     758:	000d8613          	mv	a2,s11
     75c:	02000593          	li	a1,32
     760:	00090513          	mv	a0,s2
     764:	b89ff0ef          	jal	ra,2ec <_charpad>
     768:	1aad9863          	bne	s11,a0,918 <.L24>
     76c:	00000d93          	li	s11,0

0000000000000770 <.L64>:
     770:	00005797          	auipc	a5,0x5
     774:	f7078793          	addi	a5,a5,-144 # 56e0 <prefix.0>
     778:	00090593          	mv	a1,s2
     77c:	01978533          	add	a0,a5,s9
     780:	2b1000ef          	jal	ra,1230 <__GI_fputs>
     784:	00040613          	mv	a2,s0
     788:	03000593          	li	a1,48
     78c:	00090513          	mv	a0,s2
     790:	b5dff0ef          	jal	ra,2ec <_charpad>
     794:	18a41263          	bne	s0,a0,918 <.L24>
     798:	180b9463          	bnez	s7,920 <.L65>

000000000000079c <.L66>:
     79c:	000d8613          	mv	a2,s11
     7a0:	02000593          	li	a1,32
     7a4:	00090513          	mv	a0,s2
     7a8:	b45ff0ef          	jal	ra,2ec <_charpad>
     7ac:	22ad8a63          	beq	s11,a0,9e0 <.L31>
     7b0:	1680006f          	j	918 <.L24>

00000000000007b4 <.L32>:
     7b4:	00f00713          	li	a4,15
     7b8:	04f76863          	bltu	a4,a5,808 <.L48>
     7bc:	0e412703          	lw	a4,228(sp)
     7c0:	02813783          	ld	a5,40(sp)
     7c4:	01a77733          	and	a4,a4,s10
     7c8:	02070863          	beqz	a4,7f8 <.L49>
     7cc:	0007b503          	ld	a0,0(a5)
     7d0:	0087b603          	ld	a2,8(a5)

00000000000007d4 <.L50>:
     7d4:	00050593          	mv	a1,a0
     7d8:	00000717          	auipc	a4,0x0
     7dc:	b8470713          	addi	a4,a4,-1148 # 35c <_fp_out_narrow>
     7e0:	000b0693          	mv	a3,s6
     7e4:	00090513          	mv	a0,s2
     7e8:	08d010ef          	jal	ra,2074 <_fpmaxtostr>

00000000000007ec <.L109>:
     7ec:	12054663          	bltz	a0,918 <.L24>
     7f0:	009504bb          	addw	s1,a0,s1
     7f4:	1ec0006f          	j	9e0 <.L31>

00000000000007f8 <.L49>:
     7f8:	0007b507          	fld	fa0,0(a5)
     7fc:	175040ef          	jal	ra,5170 <__extenddftf2>
     800:	00058613          	mv	a2,a1
     804:	fd1ff06f          	j	7d4 <.L50>

0000000000000808 <.L48>:
     808:	01100713          	li	a4,17
     80c:	10f77663          	bgeu	a4,a5,918 <.L24>
     810:	01300693          	li	a3,19
     814:	08f6e663          	bltu	a3,a5,8a0 <.L52>
     818:	02813703          	ld	a4,40(sp)
     81c:	06d79063          	bne	a5,a3,87c <.L53>
     820:	00073c03          	ld	s8,0(a4)
     824:	0d812683          	lw	a3,216(sp)
     828:	020c0663          	beqz	s8,854 <.L54>

000000000000082c <.L55>:
     82c:	0d812783          	lw	a5,216(sp)
     830:	fff00593          	li	a1,-1
     834:	0007c463          	bltz	a5,83c <.L56>
     838:	00078593          	mv	a1,a5

000000000000083c <.L56>:
     83c:	000c0513          	mv	a0,s8
     840:	67d000ef          	jal	ra,16bc <__GI_strnlen>
     844:	00050b93          	mv	s7,a0
     848:	02000893          	li	a7,32
     84c:	00b00c93          	li	s9,11
     850:	e35ff06f          	j	684 <.L84>

0000000000000854 <.L54>:
     854:	00500793          	li	a5,5
     858:	00006c17          	auipc	s8,0x6
     85c:	098c0c13          	addi	s8,s8,152 # 68f0 <.LC1>
     860:	02000893          	li	a7,32
     864:	00b00c93          	li	s9,11
     868:	00000413          	li	s0,0
     86c:	00000b93          	li	s7,0
     870:	e8d7fae3          	bgeu	a5,a3,704 <.L47>
     874:	00600b93          	li	s7,6
     878:	e8dff06f          	j	704 <.L47>

000000000000087c <.L53>:
     87c:	00072783          	lw	a5,0(a4)
     880:	05010c13          	addi	s8,sp,80
     884:	040108a3          	sb	zero,81(sp)
     888:	04f10823          	sb	a5,80(sp)
     88c:	02000893          	li	a7,32
     890:	00b00c93          	li	s9,11
     894:	00000413          	li	s0,0
     898:	00100b93          	li	s7,1
     89c:	e69ff06f          	j	704 <.L47>

00000000000008a0 <.L52>:
     8a0:	01400713          	li	a4,20
     8a4:	02e79263          	bne	a5,a4,8c8 <.L57>
     8a8:	00005797          	auipc	a5,0x5
     8ac:	d807b783          	ld	a5,-640(a5) # 5628 <_GLOBAL_OFFSET_TABLE_+0x18>
     8b0:	0007a503          	lw	a0,0(a5)
     8b4:	08000613          	li	a2,128
     8b8:	05010593          	addi	a1,sp,80
     8bc:	7ad000ef          	jal	ra,1868 <__GI___glibc_strerror_r>
     8c0:	00050c13          	mv	s8,a0
     8c4:	f69ff06f          	j	82c <.L55>

00000000000008c8 <.L57>:
     8c8:	00006797          	auipc	a5,0x6
     8cc:	1d878793          	addi	a5,a5,472 # 6aa0 <_custom_printf_spec>
     8d0:	0007b703          	ld	a4,0(a5)
     8d4:	0e012583          	lw	a1,224(sp)
     8d8:	00a70693          	addi	a3,a4,10
     8dc:	00070793          	mv	a5,a4

00000000000008e0 <.L59>:
     8e0:	0007c503          	lbu	a0,0(a5)
     8e4:	02b51663          	bne	a0,a1,910 <.L58>
     8e8:	40e787bb          	subw	a5,a5,a4
     8ec:	00379793          	slli	a5,a5,0x3
     8f0:	00008717          	auipc	a4,0x8
     8f4:	2d070713          	addi	a4,a4,720 # 8bc0 <_custom_printf_handler>
     8f8:	00f707b3          	add	a5,a4,a5
     8fc:	0007b783          	ld	a5,0(a5)
     900:	000b0593          	mv	a1,s6
     904:	00090513          	mv	a0,s2
     908:	000780e7          	jalr	a5
     90c:	ee1ff06f          	j	7ec <.L109>

0000000000000910 <.L58>:
     910:	00178793          	addi	a5,a5,1
     914:	fcd796e3          	bne	a5,a3,8e0 <.L59>

0000000000000918 <.L24>:
     918:	fff00493          	li	s1,-1
     91c:	b71ff06f          	j	48c <.L89>

0000000000000920 <.L65>:
     920:	00090613          	mv	a2,s2
     924:	000b8593          	mv	a1,s7
     928:	000c0513          	mv	a0,s8
     92c:	3a0010ef          	jal	ra,1ccc <__stdio_fwrite>
     930:	e6ab86e3          	beq	s7,a0,79c <.L66>
     934:	fe5ff06f          	j	918 <.L24>

0000000000000938 <.L21>:
     938:	b4a40ae3          	beq	s0,a0,48c <.L89>

000000000000093c <.L68>:
     93c:	40a40db3          	sub	s11,s0,a0
     940:	fdb05ce3          	blez	s11,918 <.L24>
     944:	00090613          	mv	a2,s2
     948:	000d8593          	mv	a1,s11
     94c:	380010ef          	jal	ra,1ccc <__stdio_fwrite>
     950:	0005051b          	sext.w	a0,a0
     954:	fcad92e3          	bne	s11,a0,918 <.L24>
     958:	00044783          	lbu	a5,0(s0)
     95c:	009d84bb          	addw	s1,s11,s1
     960:	b20786e3          	beqz	a5,48c <.L89>

0000000000000964 <.L69>:
     964:	00144703          	lbu	a4,1(s0)
     968:	02500793          	li	a5,37
     96c:	00140513          	addi	a0,s0,1
     970:	00240413          	addi	s0,s0,2
     974:	b6f70ee3          	beq	a4,a5,4f0 <.L20>
     978:	0ca13823          	sd	a0,208(sp)
     97c:	00098513          	mv	a0,s3
     980:	3c8000ef          	jal	ra,d48 <_ppfs_parsespec>
     984:	00098513          	mv	a0,s3
     988:	190000ef          	jal	ra,b18 <_ppfs_setargs>
     98c:	0ec12683          	lw	a3,236(sp)
     990:	0fa14703          	lbu	a4,250(sp)
     994:	000a8793          	mv	a5,s5
     998:	00d05863          	blez	a3,9a8 <.L27>
     99c:	00471793          	slli	a5,a4,0x4
     9a0:	ff078793          	addi	a5,a5,-16
     9a4:	00fa87b3          	add	a5,s5,a5

00000000000009a8 <.L27>:
     9a8:	0f012503          	lw	a0,240(sp)
     9ac:	02810693          	addi	a3,sp,40
     9b0:	00000593          	li	a1,0
     9b4:	00068613          	mv	a2,a3

00000000000009b8 <.L28>:
     9b8:	b4a5cce3          	blt	a1,a0,510 <.L29>
     9bc:	0f412783          	lw	a5,244(sp)
     9c0:	b60792e3          	bnez	a5,524 <.L30>
     9c4:	0e412583          	lw	a1,228(sp)
     9c8:	f00a0793          	addi	a5,s4,-256
     9cc:	00048613          	mv	a2,s1
     9d0:	00f5f5b3          	and	a1,a1,a5
     9d4:	02813783          	ld	a5,40(sp)
     9d8:	0007b503          	ld	a0,0(a5)
     9dc:	590010ef          	jal	ra,1f6c <_store_inttype>

00000000000009e0 <.L31>:
     9e0:	0d013403          	ld	s0,208(sp)
     9e4:	00040513          	mv	a0,s0
     9e8:	b09ff06f          	j	4f0 <.L20>

00000000000009ec <_ppfs_init>:
     9ec:	fd010113          	addi	sp,sp,-48
     9f0:	00913c23          	sd	s1,24(sp)
     9f4:	0f000613          	li	a2,240
     9f8:	00058493          	mv	s1,a1
     9fc:	00000593          	li	a1,0
     a00:	02813023          	sd	s0,32(sp)
     a04:	01213823          	sd	s2,16(sp)
     a08:	00050413          	mv	s0,a0
     a0c:	02113423          	sd	ra,40(sp)
     a10:	01313423          	sd	s3,8(sp)
     a14:	5a5000ef          	jal	ra,17b8 <__GI_memset>
     a18:	01c42783          	lw	a5,28(s0)
     a1c:	02c40913          	addi	s2,s0,44
     a20:	00943023          	sd	s1,0(s0)
     a24:	fff7879b          	addiw	a5,a5,-1
     a28:	00f42e23          	sw	a5,28(s0)
     a2c:	05040713          	addi	a4,s0,80
     a30:	00090793          	mv	a5,s2
     a34:	00800693          	li	a3,8

0000000000000a38 <.L2>:
     a38:	00478793          	addi	a5,a5,4
     a3c:	fed7ae23          	sw	a3,-4(a5)
     a40:	fee79ce3          	bne	a5,a4,a38 <.L2>
     a44:	00048793          	mv	a5,s1
     a48:	02500993          	li	s3,37

0000000000000a4c <.L3>:
     a4c:	0007c703          	lbu	a4,0(a5)
     a50:	02071063          	bnez	a4,a70 <.L8>
     a54:	01c42783          	lw	a5,28(s0)
     a58:	00943023          	sd	s1,0(s0)
     a5c:	00800713          	li	a4,8

0000000000000a60 <.L9>:
     a60:	fff7879b          	addiw	a5,a5,-1
     a64:	0407de63          	bgez	a5,ac0 <.L11>
     a68:	00000513          	li	a0,0
     a6c:	0280006f          	j	a94 <.L6>

0000000000000a70 <.L8>:
     a70:	0017c683          	lbu	a3,1(a5)
     a74:	05371263          	bne	a4,s3,ab8 <.L4>
     a78:	00178793          	addi	a5,a5,1
     a7c:	03368e63          	beq	a3,s3,ab8 <.L4>
     a80:	00f43023          	sd	a5,0(s0)
     a84:	00040513          	mv	a0,s0
     a88:	2c0000ef          	jal	ra,d48 <_ppfs_parsespec>
     a8c:	02055263          	bgez	a0,ab0 <.L5>

0000000000000a90 <.L10>:
     a90:	fff00513          	li	a0,-1

0000000000000a94 <.L6>:
     a94:	02813083          	ld	ra,40(sp)
     a98:	02013403          	ld	s0,32(sp)
     a9c:	01813483          	ld	s1,24(sp)
     aa0:	01013903          	ld	s2,16(sp)
     aa4:	00813983          	ld	s3,8(sp)
     aa8:	03010113          	addi	sp,sp,48
     aac:	00008067          	ret

0000000000000ab0 <.L5>:
     ab0:	00043783          	ld	a5,0(s0)
     ab4:	f99ff06f          	j	a4c <.L3>

0000000000000ab8 <.L4>:
     ab8:	00178793          	addi	a5,a5,1
     abc:	f91ff06f          	j	a4c <.L3>

0000000000000ac0 <.L11>:
     ac0:	00092683          	lw	a3,0(s2)
     ac4:	fce686e3          	beq	a3,a4,a90 <.L10>
     ac8:	00490913          	addi	s2,s2,4
     acc:	f95ff06f          	j	a60 <.L9>

0000000000000ad0 <_ppfs_prepargs>:
     ad0:	fe010113          	addi	sp,sp,-32
     ad4:	00913423          	sd	s1,8(sp)
     ad8:	01c52483          	lw	s1,28(a0)
     adc:	00113c23          	sd	ra,24(sp)
     ae0:	00813823          	sd	s0,16(sp)
     ae4:	04b53823          	sd	a1,80(a0)
     ae8:	00905e63          	blez	s1,b04 <.L1>
     aec:	00050413          	mv	s0,a0
     af0:	02952023          	sw	s1,32(a0)
     af4:	00052e23          	sw	zero,28(a0)
     af8:	00053423          	sd	zero,8(a0)
     afc:	01c000ef          	jal	ra,b18 <_ppfs_setargs>
     b00:	00942e23          	sw	s1,28(s0)

0000000000000b04 <.L1>:
     b04:	01813083          	ld	ra,24(sp)
     b08:	01013403          	ld	s0,16(sp)
     b0c:	00813483          	ld	s1,8(sp)
     b10:	02010113          	addi	sp,sp,32
     b14:	00008067          	ret

0000000000000b18 <_ppfs_setargs>:
     b18:	01c52603          	lw	a2,28(a0)
     b1c:	00c52703          	lw	a4,12(a0)
     b20:	00852683          	lw	a3,8(a0)
     b24:	06050793          	addi	a5,a0,96
     b28:	18061463          	bnez	a2,cb0 <.L2>
     b2c:	ff010113          	addi	sp,sp,-16
     b30:	00813423          	sd	s0,8(sp)
     b34:	00913023          	sd	s1,0(sp)
     b38:	800005b7          	lui	a1,0x80000
     b3c:	00b71e63          	bne	a4,a1,b58 <.L3>
     b40:	05053703          	ld	a4,80(a0)
     b44:	00870593          	addi	a1,a4,8
     b48:	00072703          	lw	a4,0(a4)
     b4c:	04b53823          	sd	a1,80(a0)
     b50:	06e52023          	sw	a4,96(a0)
     b54:	00e52623          	sw	a4,12(a0)

0000000000000b58 <.L3>:
     b58:	80000737          	lui	a4,0x80000
     b5c:	00e69e63          	bne	a3,a4,b78 <.L4>
     b60:	05053703          	ld	a4,80(a0)
     b64:	00870693          	addi	a3,a4,8 # ffffffff80000008 <_ebss+0xffffffff7fff73d8>
     b68:	00072703          	lw	a4,0(a4)
     b6c:	04d53823          	sd	a3,80(a0)
     b70:	06e52023          	sw	a4,96(a0)
     b74:	00e52423          	sw	a4,8(a0)

0000000000000b78 <.L4>:
     b78:	02052e83          	lw	t4,32(a0)
     b7c:	00001737          	lui	a4,0x1
     b80:	02c50893          	addi	a7,a0,44
     b84:	10000313          	li	t1,256
     b88:	80070f13          	addi	t5,a4,-2048 # 800 <.L49+0x8>
     b8c:	80770813          	addi	a6,a4,-2041
     b90:	20000f93          	li	t6,512
     b94:	40000293          	li	t0,1024
     b98:	00700e13          	li	t3,7
     b9c:	00800393          	li	t2,8
     ba0:	00100413          	li	s0,1
     ba4:	00200493          	li	s1,2

0000000000000ba8 <.L5>:
     ba8:	03d64e63          	blt	a2,t4,be4 <.L19>
     bac:	00c52783          	lw	a5,12(a0)
     bb0:	0207d263          	bgez	a5,bd4 <.L1>
     bb4:	40f007bb          	negw	a5,a5
     bb8:	00f52623          	sw	a5,12(a0)
     bbc:	01452783          	lw	a5,20(a0)
     bc0:	ffe7f793          	andi	a5,a5,-2
     bc4:	0087e793          	ori	a5,a5,8
     bc8:	00f52a23          	sw	a5,20(a0)
     bcc:	02000793          	li	a5,32
     bd0:	00f52c23          	sw	a5,24(a0)

0000000000000bd4 <.L1>:
     bd4:	00813403          	ld	s0,8(sp)
     bd8:	00013483          	ld	s1,0(sp)
     bdc:	01010113          	addi	sp,sp,16
     be0:	00008067          	ret

0000000000000be4 <.L19>:
     be4:	0008a703          	lw	a4,0(a7)
     be8:	0016061b          	addiw	a2,a2,1
     bec:	00670c63          	beq	a4,t1,c04 <.L12>
     bf0:	04e34a63          	blt	t1,a4,c44 <.L7>
     bf4:	0bc70263          	beq	a4,t3,c98 <.L8>
     bf8:	04ee4063          	blt	t3,a4,c38 <.L9>
     bfc:	02e44263          	blt	s0,a4,c20 <.L10>
     c00:	02074263          	bltz	a4,c24 <.L11>

0000000000000c04 <.L12>:
     c04:	05053703          	ld	a4,80(a0)
     c08:	00870693          	addi	a3,a4,8
     c0c:	00072703          	lw	a4,0(a4)
     c10:	04d53823          	sd	a3,80(a0)
     c14:	00e7a023          	sw	a4,0(a5)

0000000000000c18 <.L18>:
     c18:	01078793          	addi	a5,a5,16
     c1c:	0200006f          	j	c3c <.L13>

0000000000000c20 <.L10>:
     c20:	fe9702e3          	beq	a4,s1,c04 <.L12>

0000000000000c24 <.L11>:
     c24:	05053703          	ld	a4,80(a0)
     c28:	00870693          	addi	a3,a4,8
     c2c:	04d53823          	sd	a3,80(a0)
     c30:	00073703          	ld	a4,0(a4)
     c34:	0340006f          	j	c68 <.L30>

0000000000000c38 <.L9>:
     c38:	fe7716e3          	bne	a4,t2,c24 <.L11>

0000000000000c3c <.L13>:
     c3c:	00488893          	addi	a7,a7,4
     c40:	f69ff06f          	j	ba8 <.L5>

0000000000000c44 <.L7>:
     c44:	8007059b          	addiw	a1,a4,-2048
     c48:	05053683          	ld	a3,80(a0)
     c4c:	00058863          	beqz	a1,c5c <.L16>
     c50:	02ef4063          	blt	t5,a4,c70 <.L15>
     c54:	fbf708e3          	beq	a4,t6,c04 <.L12>
     c58:	fc5716e3          	bne	a4,t0,c24 <.L11>

0000000000000c5c <.L16>:
     c5c:	00868713          	addi	a4,a3,8
     c60:	04e53823          	sd	a4,80(a0)
     c64:	0006b703          	ld	a4,0(a3)

0000000000000c68 <.L30>:
     c68:	00e7b023          	sd	a4,0(a5)
     c6c:	fadff06f          	j	c18 <.L18>

0000000000000c70 <.L15>:
     c70:	fb071ae3          	bne	a4,a6,c24 <.L11>
     c74:	00f68693          	addi	a3,a3,15
     c78:	ff06f693          	andi	a3,a3,-16
     c7c:	01068713          	addi	a4,a3,16
     c80:	04e53823          	sd	a4,80(a0)
     c84:	0006b583          	ld	a1,0(a3)
     c88:	0086b703          	ld	a4,8(a3)
     c8c:	00b7b023          	sd	a1,0(a5)
     c90:	00e7b423          	sd	a4,8(a5)
     c94:	f85ff06f          	j	c18 <.L18>

0000000000000c98 <.L8>:
     c98:	05053703          	ld	a4,80(a0)
     c9c:	00073787          	fld	fa5,0(a4)
     ca0:	00870693          	addi	a3,a4,8
     ca4:	04d53823          	sd	a3,80(a0)
     ca8:	00f7b027          	fsd	fa5,0(a5)
     cac:	f6dff06f          	j	c18 <.L18>

0000000000000cb0 <.L2>:
     cb0:	80000637          	lui	a2,0x80000
     cb4:	00c71c63          	bne	a4,a2,ccc <.L21>
     cb8:	02854703          	lbu	a4,40(a0)
     cbc:	00471713          	slli	a4,a4,0x4
     cc0:	00e78733          	add	a4,a5,a4
     cc4:	ff072703          	lw	a4,-16(a4)
     cc8:	00e52623          	sw	a4,12(a0)

0000000000000ccc <.L21>:
     ccc:	80000737          	lui	a4,0x80000
     cd0:	00e69c63          	bne	a3,a4,ce8 <.L25>
     cd4:	02954703          	lbu	a4,41(a0)
     cd8:	00471713          	slli	a4,a4,0x4
     cdc:	00e787b3          	add	a5,a5,a4
     ce0:	ff07a783          	lw	a5,-16(a5)
     ce4:	00f52423          	sw	a5,8(a0)

0000000000000ce8 <.L25>:
     ce8:	00c52783          	lw	a5,12(a0)
     cec:	0207d463          	bgez	a5,d14 <.L27>
     cf0:	40f007bb          	negw	a5,a5
     cf4:	00f52623          	sw	a5,12(a0)
     cf8:	01452783          	lw	a5,20(a0)
     cfc:	ffe7f793          	andi	a5,a5,-2
     d00:	0087e793          	ori	a5,a5,8
     d04:	00f52a23          	sw	a5,20(a0)
     d08:	02000793          	li	a5,32
     d0c:	00f52c23          	sw	a5,24(a0)
     d10:	00008067          	ret

0000000000000d14 <.L27>:
     d14:	00008067          	ret

0000000000000d18 <_promoted_size>:
     d18:	82818793          	addi	a5,gp,-2008 # 5708 <type_sizes>
     d1c:	81018713          	addi	a4,gp,-2032 # 56f0 <type_codes>

0000000000000d20 <.L3>:
     d20:	ffe79683          	lh	a3,-2(a5)
     d24:	ffe78793          	addi	a5,a5,-2
     d28:	00a68463          	beq	a3,a0,d30 <.L2>
     d2c:	fee79ae3          	bne	a5,a4,d20 <.L3>

0000000000000d30 <.L2>:
     d30:	40e787b3          	sub	a5,a5,a4
     d34:	4017d793          	srai	a5,a5,0x1
     d38:	0007879b          	sext.w	a5,a5
     d3c:	00f70733          	add	a4,a4,a5
     d40:	01874503          	lbu	a0,24(a4) # ffffffff80000018 <_ebss+0xffffffff7fff73e8>
     d44:	00008067          	ret

0000000000000d48 <_ppfs_parsespec>:
     d48:	00800793          	li	a5,8
     d4c:	02079793          	slli	a5,a5,0x20
     d50:	f6010113          	addi	sp,sp,-160
     d54:	0cccd837          	lui	a6,0xcccd
     d58:	00878793          	addi	a5,a5,8
     d5c:	800008b7          	lui	a7,0x80000
     d60:	00053683          	ld	a3,0(a0)
     d64:	08813823          	sd	s0,144(sp)
     d68:	09213023          	sd	s2,128(sp)
     d6c:	07313c23          	sd	s3,120(sp)
     d70:	01c52903          	lw	s2,28(a0)
     d74:	07413823          	sd	s4,112(sp)
     d78:	07513423          	sd	s5,104(sp)
     d7c:	07613023          	sd	s6,96(sp)
     d80:	05713c23          	sd	s7,88(sp)
     d84:	00050413          	mv	s0,a0
     d88:	ccb80993          	addi	s3,a6,-821 # ccccccb <_ebss+0xccc409b>
     d8c:	08113c23          	sd	ra,152(sp)
     d90:	08913423          	sd	s1,136(sp)
     d94:	05813823          	sd	s8,80(sp)
     d98:	05913423          	sd	s9,72(sp)
     d9c:	05a13023          	sd	s10,64(sp)
     da0:	03b13c23          	sd	s11,56(sp)
     da4:	00000313          	li	t1,0
     da8:	00013023          	sd	zero,0(sp)
     dac:	00f13823          	sd	a5,16(sp)
     db0:	00006397          	auipc	t2,0x6
     db4:	d103b383          	ld	t2,-752(t2) # 6ac0 <__GI___ctype_b>
     db8:	00000e93          	li	t4,0
     dbc:	00000513          	li	a0,0
     dc0:	00000613          	li	a2,0
     dc4:	00000e13          	li	t3,0
     dc8:	02a00f13          	li	t5,42
     dcc:	00a00a13          	li	s4,10
     dd0:	ccc80813          	addi	a6,a6,-820
     dd4:	fff8c893          	not	a7,a7
     dd8:	03700a93          	li	s5,55
     ddc:	02500f93          	li	t6,37
     de0:	02e00b13          	li	s6,46
     de4:	02400293          	li	t0,36
     de8:	03000b93          	li	s7,48

0000000000000dec <.L6>:
     dec:	0006c783          	lbu	a5,0(a3)
     df0:	00068713          	mv	a4,a3
     df4:	01e79c63          	bne	a5,t5,e0c <.L7>
     df8:	03010713          	addi	a4,sp,48
     dfc:	00251793          	slli	a5,a0,0x2
     e00:	40f707b3          	sub	a5,a4,a5
     e04:	fe07a023          	sw	zero,-32(a5)
     e08:	00168713          	addi	a4,a3,1

0000000000000e0c <.L7>:
     e0c:	00000793          	li	a5,0

0000000000000e10 <.L8>:
     e10:	00074583          	lbu	a1,0(a4)
     e14:	00159493          	slli	s1,a1,0x1
     e18:	009384b3          	add	s1,t2,s1
     e1c:	0004d483          	lhu	s1,0(s1)
     e20:	0084f493          	andi	s1,s1,8
     e24:	04049e63          	bnez	s1,e80 <.L11>
     e28:	fff6c483          	lbu	s1,-1(a3)
     e2c:	0df49663          	bne	s1,t6,ef8 <.L12>
     e30:	0a559e63          	bne	a1,t0,eec <.L13>
     e34:	3ef05263          	blez	a5,1218 <.L14>
     e38:	00170693          	addi	a3,a4,1
     e3c:	06091663          	bnez	s2,ea8 <.L15>

0000000000000e40 <.L20>:
     e40:	fff00513          	li	a0,-1

0000000000000e44 <.L79>:
     e44:	09813083          	ld	ra,152(sp)
     e48:	09013403          	ld	s0,144(sp)
     e4c:	08813483          	ld	s1,136(sp)
     e50:	08013903          	ld	s2,128(sp)
     e54:	07813983          	ld	s3,120(sp)
     e58:	07013a03          	ld	s4,112(sp)
     e5c:	06813a83          	ld	s5,104(sp)
     e60:	06013b03          	ld	s6,96(sp)
     e64:	05813b83          	ld	s7,88(sp)
     e68:	05013c03          	ld	s8,80(sp)
     e6c:	04813c83          	ld	s9,72(sp)
     e70:	04013d03          	ld	s10,64(sp)
     e74:	03813d83          	ld	s11,56(sp)
     e78:	0a010113          	addi	sp,sp,160
     e7c:	00008067          	ret

0000000000000e80 <.L11>:
     e80:	00f9d663          	bge	s3,a5,e8c <.L9>
     e84:	01079e63          	bne	a5,a6,ea0 <.L57>
     e88:	00baec63          	bltu	s5,a1,ea0 <.L57>

0000000000000e8c <.L9>:
     e8c:	02fa07bb          	mulw	a5,s4,a5
     e90:	fd05859b          	addiw	a1,a1,-48 # 7fffffd0 <_ebss+0x7fff73a0>
     e94:	00f587bb          	addw	a5,a1,a5

0000000000000e98 <.L10>:
     e98:	00170713          	addi	a4,a4,1
     e9c:	f75ff06f          	j	e10 <.L8>

0000000000000ea0 <.L57>:
     ea0:	00088793          	mv	a5,a7
     ea4:	ff5ff06f          	j	e98 <.L10>

0000000000000ea8 <.L15>:
     ea8:	00078713          	mv	a4,a5
     eac:	0127d463          	bge	a5,s2,eb4 <.L17>
     eb0:	00090713          	mv	a4,s2

0000000000000eb4 <.L17>:
     eb4:	0007091b          	sext.w	s2,a4
     eb8:	00078313          	mv	t1,a5
     ebc:	00100e93          	li	t4,1

0000000000000ec0 <.L18>:
     ec0:	00068713          	mv	a4,a3

0000000000000ec4 <.L22>:
     ec4:	00074483          	lbu	s1,0(a4)
     ec8:	00100793          	li	a5,1
     ecc:	00006697          	auipc	a3,0x6
     ed0:	a4c68693          	addi	a3,a3,-1460 # 6918 <spec_flags.5>

0000000000000ed4 <.L23>:
     ed4:	0006c583          	lbu	a1,0(a3)
     ed8:	00168693          	addi	a3,a3,1
     edc:	22959463          	bne	a1,s1,1104 <.L21>
     ee0:	00170713          	addi	a4,a4,1
     ee4:	00f66633          	or	a2,a2,a5
     ee8:	fddff06f          	j	ec4 <.L22>

0000000000000eec <.L13>:
     eec:	33205863          	blez	s2,121c <.L19>
     ef0:	06d00493          	li	s1,109
     ef4:	f49596e3          	bne	a1,s1,e40 <.L20>

0000000000000ef8 <.L12>:
     ef8:	0006c583          	lbu	a1,0(a3)
     efc:	00070493          	mv	s1,a4
     f00:	03e59663          	bne	a1,t5,f2c <.L25>
     f04:	22090663          	beqz	s2,1130 <.L26>
     f08:	00170493          	addi	s1,a4,1
     f0c:	00074703          	lbu	a4,0(a4)
     f10:	f25718e3          	bne	a4,t0,e40 <.L20>
     f14:	f2f056e3          	blez	a5,e40 <.L20>
     f18:	00251713          	slli	a4,a0,0x2
     f1c:	03010693          	addi	a3,sp,48
     f20:	40e68733          	sub	a4,a3,a4
     f24:	fcf72823          	sw	a5,-48(a4)

0000000000000f28 <.L81>:
     f28:	800007b7          	lui	a5,0x80000

0000000000000f2c <.L25>:
     f2c:	0004c583          	lbu	a1,0(s1)
     f30:	20050663          	beqz	a0,113c <.L27>
     f34:	000e8463          	beqz	t4,f3c <.L29>
     f38:	00612423          	sw	t1,8(sp)

0000000000000f3c <.L29>:
     f3c:	83818713          	addi	a4,gp,-1992 # 5718 <qual_chars.4>
     f40:	00070693          	mv	a3,a4

0000000000000f44 <.L34>:
     f44:	00074503          	lbu	a0,0(a4)
     f48:	22b51063          	bne	a0,a1,1168 <.L32>
     f4c:	00148493          	addi	s1,s1,1

0000000000000f50 <.L33>:
     f50:	40d706b3          	sub	a3,a4,a3
     f54:	00100513          	li	a0,1
     f58:	0004c583          	lbu	a1,0(s1)
     f5c:	00d54a63          	blt	a0,a3,f70 <.L35>
     f60:	00074683          	lbu	a3,0(a4)
     f64:	00b69663          	bne	a3,a1,f70 <.L35>
     f68:	00970713          	addi	a4,a4,9
     f6c:	00148493          	addi	s1,s1,1

0000000000000f70 <.L35>:
     f70:	00974683          	lbu	a3,9(a4)
     f74:	0004c583          	lbu	a1,0(s1)
     f78:	0086969b          	slliw	a3,a3,0x8
     f7c:	ec0582e3          	beqz	a1,e40 <.L20>
     f80:	00006717          	auipc	a4,0x6
     f84:	98070713          	addi	a4,a4,-1664 # 6900 <spec_chars.3>
     f88:	00070813          	mv	a6,a4

0000000000000f8c <.L40>:
     f8c:	00074503          	lbu	a0,0(a4)
     f90:	1eb51463          	bne	a0,a1,1178 <.L36>
     f94:	4107073b          	subw	a4,a4,a6
     f98:	01100593          	li	a1,17
     f9c:	00e5d863          	bge	a1,a4,fac <.L37>
     fa0:	4006f593          	andi	a1,a3,1024
     fa4:	00058463          	beqz	a1,fac <.L37>
     fa8:	ffe7071b          	addiw	a4,a4,-2

0000000000000fac <.L37>:
     fac:	85018513          	addi	a0,gp,-1968 # 5730 <spec_ranges.2>
     fb0:	02e42223          	sw	a4,36(s0)
     fb4:	00050813          	mv	a6,a0

0000000000000fb8 <.L38>:
     fb8:	00050593          	mv	a1,a0
     fbc:	0005c883          	lbu	a7,0(a1)
     fc0:	00150513          	addi	a0,a0,1
     fc4:	fee8cae3          	blt	a7,a4,fb8 <.L38>
     fc8:	410585bb          	subw	a1,a1,a6
     fcc:	81018713          	addi	a4,gp,-2032 # 56f0 <type_codes>
     fd0:	00159593          	slli	a1,a1,0x1
     fd4:	00b705b3          	add	a1,a4,a1
     fd8:	05059703          	lh	a4,80(a1)
     fdc:	06059583          	lh	a1,96(a1)
     fe0:	00e6e733          	or	a4,a3,a4
     fe4:	00b77733          	and	a4,a4,a1
     fe8:	00e12c23          	sw	a4,24(sp)
     fec:	00006717          	auipc	a4,0x6
     ff0:	91470713          	addi	a4,a4,-1772 # 6900 <spec_chars.3>

0000000000000ff4 <.L39>:
     ff4:	0004c583          	lbu	a1,0(s1)
     ff8:	00f42423          	sw	a5,8(s0)
     ffc:	01c42623          	sw	t3,12(s0)
    1000:	00b42823          	sw	a1,16(s0)
    1004:	00467793          	andi	a5,a2,4
    1008:	03000593          	li	a1,48
    100c:	00079463          	bnez	a5,1014 <.L41>
    1010:	02000593          	li	a1,32

0000000000001014 <.L41>:
    1014:	000017b7          	lui	a5,0x1
    1018:	f0078793          	addi	a5,a5,-256 # f00 <.L12+0x8>
    101c:	00f6f6b3          	and	a3,a3,a5
    1020:	ffb67613          	andi	a2,a2,-5
    1024:	00100793          	li	a5,1
    1028:	00d66633          	or	a2,a2,a3
    102c:	00b42c23          	sw	a1,24(s0)
    1030:	00c42a23          	sw	a2,20(s0)
    1034:	02f42023          	sw	a5,32(s0)
    1038:	00074783          	lbu	a5,0(a4)
    103c:	00079e63          	bnez	a5,1058 <.L44>
    1040:	0004c703          	lbu	a4,0(s1)
    1044:	06d00793          	li	a5,109
    1048:	14f71063          	bne	a4,a5,1188 <.L43>
    104c:	00500793          	li	a5,5
    1050:	02279793          	slli	a5,a5,0x22
    1054:	02f43023          	sd	a5,32(s0)

0000000000001058 <.L44>:
    1058:	1b205063          	blez	s2,11f8 <.L47>
    105c:	00812c03          	lw	s8,8(sp)
    1060:	01010b93          	addi	s7,sp,16
    1064:	00010b13          	mv	s6,sp
    1068:	00000a13          	li	s4,0
    106c:	00200d93          	li	s11,2
    1070:	ffec0c1b          	addiw	s8,s8,-2
    1074:	00900693          	li	a3,9
    1078:	00800d13          	li	s10,8

000000000000107c <.L53>:
    107c:	174dea63          	bltu	s11,s4,11f0 <.L48>
    1080:	000b2783          	lw	a5,0(s6)
    1084:	01440733          	add	a4,s0,s4
    1088:	02f70423          	sb	a5,40(a4)
    108c:	0ff7f793          	zext.b	a5,a5

0000000000001090 <.L49>:
    1090:	00f95663          	bge	s2,a5,109c <.L50>
    1094:	daf6c6e3          	blt	a3,a5,e40 <.L20>
    1098:	00078913          	mv	s2,a5

000000000000109c <.L50>:
    109c:	000baa83          	lw	s5,0(s7)
    10a0:	03aa8a63          	beq	s5,s10,10d4 <.L51>
    10a4:	00279793          	slli	a5,a5,0x2
    10a8:	00f409b3          	add	s3,s0,a5
    10ac:	0289a503          	lw	a0,40(s3)
    10b0:	03a50063          	beq	a0,s10,10d0 <.L52>
    10b4:	01550e63          	beq	a0,s5,10d0 <.L52>
    10b8:	c61ff0ef          	jal	ra,d18 <_promoted_size>
    10bc:	00050c93          	mv	s9,a0
    10c0:	000a8513          	mv	a0,s5
    10c4:	c55ff0ef          	jal	ra,d18 <_promoted_size>
    10c8:	00900693          	li	a3,9
    10cc:	01954463          	blt	a0,s9,10d4 <.L51>

00000000000010d0 <.L52>:
    10d0:	0359a423          	sw	s5,40(s3)

00000000000010d4 <.L51>:
    10d4:	02042703          	lw	a4,32(s0)
    10d8:	001a0a13          	addi	s4,s4,1
    10dc:	fffa079b          	addiw	a5,s4,-1
    10e0:	004b8b93          	addi	s7,s7,4
    10e4:	004b0b13          	addi	s6,s6,4
    10e8:	f8f75ae3          	bge	a4,a5,107c <.L53>

00000000000010ec <.L54>:
    10ec:	02042503          	lw	a0,32(s0)
    10f0:	00148493          	addi	s1,s1,1
    10f4:	01242e23          	sw	s2,28(s0)
    10f8:	00943023          	sd	s1,0(s0)
    10fc:	0025051b          	addiw	a0,a0,2
    1100:	d45ff06f          	j	e44 <.L79>

0000000000001104 <.L21>:
    1104:	0006c583          	lbu	a1,0(a3)
    1108:	0017979b          	slliw	a5,a5,0x1
    110c:	dc0594e3          	bnez	a1,ed4 <.L23>
    1110:	4016579b          	sraiw	a5,a2,0x1
    1114:	0057f793          	andi	a5,a5,5
    1118:	fff7c793          	not	a5,a5
    111c:	00c7f633          	and	a2,a5,a2
    1120:	fff74783          	lbu	a5,-1(a4)
    1124:	03f79463          	bne	a5,t6,114c <.L24>
    1128:	00000793          	li	a5,0
    112c:	dcdff06f          	j	ef8 <.L12>

0000000000001130 <.L26>:
    1130:	00168493          	addi	s1,a3,1
    1134:	de970ae3          	beq	a4,s1,f28 <.L81>
    1138:	d09ff06f          	j	e40 <.L20>

000000000000113c <.L27>:
    113c:	01659c63          	bne	a1,s6,1154 <.L30>
    1140:	00148713          	addi	a4,s1,1
    1144:	00078e13          	mv	t3,a5
    1148:	fff00513          	li	a0,-1

000000000000114c <.L24>:
    114c:	00070693          	mv	a3,a4
    1150:	c9dff06f          	j	dec <.L6>

0000000000001154 <.L30>:
    1154:	000e8463          	beqz	t4,115c <.L31>
    1158:	00612423          	sw	t1,8(sp)

000000000000115c <.L31>:
    115c:	00078e13          	mv	t3,a5
    1160:	fff00793          	li	a5,-1
    1164:	dd9ff06f          	j	f3c <.L29>

0000000000001168 <.L32>:
    1168:	00174503          	lbu	a0,1(a4)
    116c:	00170713          	addi	a4,a4,1
    1170:	dc051ae3          	bnez	a0,f44 <.L34>
    1174:	dddff06f          	j	f50 <.L33>

0000000000001178 <.L36>:
    1178:	00174503          	lbu	a0,1(a4)
    117c:	00170713          	addi	a4,a4,1
    1180:	e00516e3          	bnez	a0,f8c <.L40>
    1184:	e71ff06f          	j	ff4 <.L39>

0000000000001188 <.L43>:
    1188:	01500793          	li	a5,21
    118c:	02f42223          	sw	a5,36(s0)
    1190:	0004c603          	lbu	a2,0(s1)
    1194:	00006717          	auipc	a4,0x6
    1198:	90c73703          	ld	a4,-1780(a4) # 6aa0 <_custom_printf_spec>
    119c:	00a70693          	addi	a3,a4,10
    11a0:	00070793          	mv	a5,a4

00000000000011a4 <.L46>:
    11a4:	0007c583          	lbu	a1,0(a5)
    11a8:	02c59e63          	bne	a1,a2,11e4 <.L45>
    11ac:	40e787bb          	subw	a5,a5,a4
    11b0:	00379793          	slli	a5,a5,0x3
    11b4:	00008717          	auipc	a4,0x8
    11b8:	9bc70713          	addi	a4,a4,-1604 # 8b70 <_custom_printf_arginfo>
    11bc:	00f707b3          	add	a5,a4,a5
    11c0:	0007b783          	ld	a5,0(a5)
    11c4:	01810613          	addi	a2,sp,24
    11c8:	00500593          	li	a1,5
    11cc:	00840513          	addi	a0,s0,8
    11d0:	000780e7          	jalr	a5
    11d4:	02a42023          	sw	a0,32(s0)
    11d8:	00500793          	li	a5,5
    11dc:	e6a7dee3          	bge	a5,a0,1058 <.L44>
    11e0:	c61ff06f          	j	e40 <.L20>

00000000000011e4 <.L45>:
    11e4:	00178793          	addi	a5,a5,1
    11e8:	fad79ee3          	bne	a5,a3,11a4 <.L46>
    11ec:	c55ff06f          	j	e40 <.L20>

00000000000011f0 <.L48>:
    11f0:	014c07bb          	addw	a5,s8,s4
    11f4:	e9dff06f          	j	1090 <.L49>

00000000000011f8 <.L47>:
    11f8:	02042603          	lw	a2,32(s0)
    11fc:	00100793          	li	a5,1
    1200:	02f40523          	sb	a5,42(s0)
    1204:	00261613          	slli	a2,a2,0x2
    1208:	01810593          	addi	a1,sp,24
    120c:	02c40513          	addi	a0,s0,44
    1210:	20c000ef          	jal	ra,141c <__GI_memcpy>
    1214:	ed9ff06f          	j	10ec <.L54>

0000000000001218 <.L14>:
    1218:	c32044e3          	bgtz	s2,e40 <.L20>

000000000000121c <.L19>:
    121c:	0006c583          	lbu	a1,0(a3)
    1220:	00000913          	li	s2,0
    1224:	c8e6fee3          	bgeu	a3,a4,ec0 <.L18>
    1228:	cd7598e3          	bne	a1,s7,ef8 <.L12>
    122c:	c95ff06f          	j	ec0 <.L18>

0000000000001230 <__GI_fputs>:
    1230:	fd010113          	addi	sp,sp,-48
    1234:	02813023          	sd	s0,32(sp)
    1238:	00913c23          	sd	s1,24(sp)
    123c:	02113423          	sd	ra,40(sp)
    1240:	00050493          	mv	s1,a0
    1244:	00b13423          	sd	a1,8(sp)
    1248:	0ec000ef          	jal	ra,1334 <__GI_strlen>
    124c:	00813683          	ld	a3,8(sp)
    1250:	00050413          	mv	s0,a0
    1254:	00050613          	mv	a2,a0
    1258:	00100593          	li	a1,1
    125c:	00048513          	mv	a0,s1
    1260:	028000ef          	jal	ra,1288 <__GI_fwrite>
    1264:	00851e63          	bne	a0,s0,1280 <.L3>
    1268:	0005051b          	sext.w	a0,a0

000000000000126c <.L2>:
    126c:	02813083          	ld	ra,40(sp)
    1270:	02013403          	ld	s0,32(sp)
    1274:	01813483          	ld	s1,24(sp)
    1278:	03010113          	addi	sp,sp,48
    127c:	00008067          	ret

0000000000001280 <.L3>:
    1280:	fff00513          	li	a0,-1
    1284:	fe9ff06f          	j	126c <.L2>

0000000000001288 <__GI_fwrite>:
    1288:	0006d783          	lhu	a5,0(a3)
    128c:	fd010113          	addi	sp,sp,-48
    1290:	02813023          	sd	s0,32(sp)
    1294:	00913c23          	sd	s1,24(sp)
    1298:	01213823          	sd	s2,16(sp)
    129c:	01313423          	sd	s3,8(sp)
    12a0:	02113423          	sd	ra,40(sp)
    12a4:	0407f793          	andi	a5,a5,64
    12a8:	00050993          	mv	s3,a0
    12ac:	00058493          	mv	s1,a1
    12b0:	00060913          	mv	s2,a2
    12b4:	00068413          	mv	s0,a3
    12b8:	00078863          	beqz	a5,12c8 <.L2>

00000000000012bc <.L5>:
    12bc:	00049e63          	bnez	s1,12d8 <.L3>

00000000000012c0 <.L20>:
    12c0:	00000513          	li	a0,0
    12c4:	0340006f          	j	12f8 <.L1>

00000000000012c8 <.L2>:
    12c8:	00068513          	mv	a0,a3
    12cc:	351000ef          	jal	ra,1e1c <__stdio_trans2w>
    12d0:	fe0506e3          	beqz	a0,12bc <.L5>
    12d4:	fedff06f          	j	12c0 <.L20>

00000000000012d8 <.L3>:
    12d8:	fe0904e3          	beqz	s2,12c0 <.L20>
    12dc:	029937b3          	mulhu	a5,s2,s1
    12e0:	02079a63          	bnez	a5,1314 <.L18>
    12e4:	032485b3          	mul	a1,s1,s2
    12e8:	00040613          	mv	a2,s0
    12ec:	00098513          	mv	a0,s3
    12f0:	1dd000ef          	jal	ra,1ccc <__stdio_fwrite>
    12f4:	02955533          	divu	a0,a0,s1

00000000000012f8 <.L1>:
    12f8:	02813083          	ld	ra,40(sp)
    12fc:	02013403          	ld	s0,32(sp)
    1300:	01813483          	ld	s1,24(sp)
    1304:	01013903          	ld	s2,16(sp)
    1308:	00813983          	ld	s3,8(sp)
    130c:	03010113          	addi	sp,sp,48
    1310:	00008067          	ret

0000000000001314 <.L18>:
    1314:	00045783          	lhu	a5,0(s0)
    1318:	01600713          	li	a4,22
    131c:	0087e793          	ori	a5,a5,8
    1320:	00f41023          	sh	a5,0(s0)
    1324:	00004797          	auipc	a5,0x4
    1328:	3047b783          	ld	a5,772(a5) # 5628 <_GLOBAL_OFFSET_TABLE_+0x18>
    132c:	00e7a023          	sw	a4,0(a5)
    1330:	f91ff06f          	j	12c0 <.L20>

0000000000001334 <__GI_strlen>:
    1334:	00050713          	mv	a4,a0

0000000000001338 <.L2>:
    1338:	00777793          	andi	a5,a4,7
    133c:	02079e63          	bnez	a5,1378 <.L5>
    1340:	00005617          	auipc	a2,0x5
    1344:	5e063603          	ld	a2,1504(a2) # 6920 <.LC0>
    1348:	00005597          	auipc	a1,0x5
    134c:	5e05b583          	ld	a1,1504(a1) # 6928 <.LC1>

0000000000001350 <.L15>:
    1350:	00070793          	mv	a5,a4
    1354:	00073683          	ld	a3,0(a4)
    1358:	00870713          	addi	a4,a4,8
    135c:	00c686b3          	add	a3,a3,a2
    1360:	00b6f6b3          	and	a3,a3,a1
    1364:	fe0686e3          	beqz	a3,1350 <.L15>
    1368:	ff874683          	lbu	a3,-8(a4)
    136c:	02069263          	bnez	a3,1390 <.L7>
    1370:	40a78533          	sub	a0,a5,a0
    1374:	00008067          	ret

0000000000001378 <.L5>:
    1378:	00074783          	lbu	a5,0(a4)
    137c:	00079663          	bnez	a5,1388 <.L3>
    1380:	40a70533          	sub	a0,a4,a0
    1384:	00008067          	ret

0000000000001388 <.L3>:
    1388:	00170713          	addi	a4,a4,1
    138c:	fadff06f          	j	1338 <.L2>

0000000000001390 <.L7>:
    1390:	ff974683          	lbu	a3,-7(a4)
    1394:	00069863          	bnez	a3,13a4 <.L8>
    1398:	40a787b3          	sub	a5,a5,a0
    139c:	00178513          	addi	a0,a5,1
    13a0:	00008067          	ret

00000000000013a4 <.L8>:
    13a4:	ffa74683          	lbu	a3,-6(a4)
    13a8:	00069863          	bnez	a3,13b8 <.L9>
    13ac:	40a787b3          	sub	a5,a5,a0
    13b0:	00278513          	addi	a0,a5,2
    13b4:	00008067          	ret

00000000000013b8 <.L9>:
    13b8:	ffb74683          	lbu	a3,-5(a4)
    13bc:	00069863          	bnez	a3,13cc <.L10>
    13c0:	40a787b3          	sub	a5,a5,a0
    13c4:	00378513          	addi	a0,a5,3
    13c8:	00008067          	ret

00000000000013cc <.L10>:
    13cc:	ffc74683          	lbu	a3,-4(a4)
    13d0:	00069863          	bnez	a3,13e0 <.L11>
    13d4:	40a787b3          	sub	a5,a5,a0
    13d8:	00478513          	addi	a0,a5,4
    13dc:	00008067          	ret

00000000000013e0 <.L11>:
    13e0:	ffd74683          	lbu	a3,-3(a4)
    13e4:	00069863          	bnez	a3,13f4 <.L12>
    13e8:	40a787b3          	sub	a5,a5,a0
    13ec:	00578513          	addi	a0,a5,5
    13f0:	00008067          	ret

00000000000013f4 <.L12>:
    13f4:	ffe74683          	lbu	a3,-2(a4)
    13f8:	00069863          	bnez	a3,1408 <.L13>
    13fc:	40a787b3          	sub	a5,a5,a0
    1400:	00678513          	addi	a0,a5,6
    1404:	00008067          	ret

0000000000001408 <.L13>:
    1408:	fff74683          	lbu	a3,-1(a4)
    140c:	f40692e3          	bnez	a3,1350 <.L15>
    1410:	40a787b3          	sub	a5,a5,a0
    1414:	00778513          	addi	a0,a5,7
    1418:	00008067          	ret

000000000000141c <__GI_memcpy>:
    141c:	00f00713          	li	a4,15
    1420:	00050693          	mv	a3,a0
    1424:	00058793          	mv	a5,a1
    1428:	0ac77a63          	bgeu	a4,a2,14dc <.L2>
    142c:	40a007b3          	neg	a5,a0
    1430:	0077f793          	andi	a5,a5,7
    1434:	40f60633          	sub	a2,a2,a5
    1438:	00000713          	li	a4,0

000000000000143c <.L3>:
    143c:	04e79663          	bne	a5,a4,1488 <.L4>
    1440:	00f585b3          	add	a1,a1,a5
    1444:	00f50333          	add	t1,a0,a5
    1448:	0075fe13          	andi	t3,a1,7
    144c:	00030713          	mv	a4,t1
    1450:	00058693          	mv	a3,a1
    1454:	00365793          	srli	a5,a2,0x3
    1458:	140e1463          	bnez	t3,15a0 <.L5>
    145c:	0077f893          	andi	a7,a5,7
    1460:	fff88893          	addi	a7,a7,-1 # ffffffff7fffffff <_ebss+0xffffffff7fff73cf>
    1464:	00600e13          	li	t3,6
    1468:	0005b803          	ld	a6,0(a1)
    146c:	111e6263          	bltu	t3,a7,1570 <.L6>
    1470:	88018e13          	addi	t3,gp,-1920 # 5760 <.L8>
    1474:	00289893          	slli	a7,a7,0x2
    1478:	01c888b3          	add	a7,a7,t3
    147c:	0008a683          	lw	a3,0(a7)
    1480:	01c686b3          	add	a3,a3,t3
    1484:	00068067          	jr	a3

0000000000001488 <.L4>:
    1488:	00e586b3          	add	a3,a1,a4
    148c:	0006c803          	lbu	a6,0(a3)
    1490:	00e506b3          	add	a3,a0,a4
    1494:	00170713          	addi	a4,a4,1
    1498:	01068023          	sb	a6,0(a3)
    149c:	fa1ff06f          	j	143c <.L3>

00000000000014a0 <.L13>:
    14a0:	fd058693          	addi	a3,a1,-48
    14a4:	fc830713          	addi	a4,t1,-56
    14a8:	00678793          	addi	a5,a5,6
    14ac:	00080893          	mv	a7,a6

00000000000014b0 <.L15>:
    14b0:	0386b803          	ld	a6,56(a3)
    14b4:	ff878793          	addi	a5,a5,-8
    14b8:	03173c23          	sd	a7,56(a4)
    14bc:	04068693          	addi	a3,a3,64
    14c0:	04070713          	addi	a4,a4,64
    14c4:	0c079663          	bnez	a5,1590 <.L23>

00000000000014c8 <.L22>:
    14c8:	01073023          	sd	a6,0(a4)

00000000000014cc <.L24>:
    14cc:	ff867693          	andi	a3,a2,-8
    14d0:	00b687b3          	add	a5,a3,a1
    14d4:	00767613          	andi	a2,a2,7
    14d8:	006686b3          	add	a3,a3,t1

00000000000014dc <.L2>:
    14dc:	00000713          	li	a4,0

00000000000014e0 <.L34>:
    14e0:	1ce61263          	bne	a2,a4,16a4 <.L35>
    14e4:	00008067          	ret

00000000000014e8 <.L12>:
    14e8:	fd858693          	addi	a3,a1,-40
    14ec:	fd030713          	addi	a4,t1,-48
    14f0:	00578793          	addi	a5,a5,5

00000000000014f4 <.L16>:
    14f4:	0306b883          	ld	a7,48(a3)
    14f8:	03073823          	sd	a6,48(a4)
    14fc:	fb5ff06f          	j	14b0 <.L15>

0000000000001500 <.L11>:
    1500:	fe058693          	addi	a3,a1,-32
    1504:	fd830713          	addi	a4,t1,-40
    1508:	00478793          	addi	a5,a5,4

000000000000150c <.L17>:
    150c:	0286b883          	ld	a7,40(a3)
    1510:	03073423          	sd	a6,40(a4)
    1514:	00088813          	mv	a6,a7
    1518:	fddff06f          	j	14f4 <.L16>

000000000000151c <.L10>:
    151c:	fe858693          	addi	a3,a1,-24
    1520:	fe030713          	addi	a4,t1,-32
    1524:	00378793          	addi	a5,a5,3

0000000000001528 <.L18>:
    1528:	0206b883          	ld	a7,32(a3)
    152c:	03073023          	sd	a6,32(a4)
    1530:	00088813          	mv	a6,a7
    1534:	fd9ff06f          	j	150c <.L17>

0000000000001538 <.L9>:
    1538:	ff058693          	addi	a3,a1,-16
    153c:	fe830713          	addi	a4,t1,-24
    1540:	00278793          	addi	a5,a5,2

0000000000001544 <.L19>:
    1544:	0186b883          	ld	a7,24(a3)
    1548:	01073c23          	sd	a6,24(a4)
    154c:	00088813          	mv	a6,a7
    1550:	fd9ff06f          	j	1528 <.L18>

0000000000001554 <.L7>:
    1554:	ff858693          	addi	a3,a1,-8
    1558:	ff030713          	addi	a4,t1,-16
    155c:	00178793          	addi	a5,a5,1

0000000000001560 <.L20>:
    1560:	0106b883          	ld	a7,16(a3)
    1564:	01073823          	sd	a6,16(a4)
    1568:	00088813          	mv	a6,a7
    156c:	fd9ff06f          	j	1544 <.L19>

0000000000001570 <.L6>:
    1570:	ff830713          	addi	a4,t1,-8

0000000000001574 <.L21>:
    1574:	0086b883          	ld	a7,8(a3)
    1578:	01073423          	sd	a6,8(a4)
    157c:	00088813          	mv	a6,a7
    1580:	fe1ff06f          	j	1560 <.L20>

0000000000001584 <.L14>:
    1584:	fff78793          	addi	a5,a5,-1
    1588:	f40780e3          	beqz	a5,14c8 <.L22>
    158c:	00858693          	addi	a3,a1,8

0000000000001590 <.L23>:
    1590:	0006b883          	ld	a7,0(a3)
    1594:	01073023          	sd	a6,0(a4)
    1598:	00088813          	mv	a6,a7
    159c:	fd9ff06f          	j	1574 <.L21>

00000000000015a0 <.L5>:
    15a0:	003e169b          	slliw	a3,t3,0x3
    15a4:	ff85f813          	andi	a6,a1,-8
    15a8:	04000f13          	li	t5,64
    15ac:	0037fe93          	andi	t4,a5,3
    15b0:	00200293          	li	t0,2
    15b4:	40df0f3b          	subw	t5,t5,a3
    15b8:	00883883          	ld	a7,8(a6)
    15bc:	00083683          	ld	a3,0(a6)
    15c0:	003e1e13          	slli	t3,t3,0x3
    15c4:	00880f93          	addi	t6,a6,8
    15c8:	045e8063          	beq	t4,t0,1608 <.L25>
    15cc:	00300293          	li	t0,3
    15d0:	085e8263          	beq	t4,t0,1654 <.L26>
    15d4:	00100293          	li	t0,1
    15d8:	0a5e9263          	bne	t4,t0,167c <.L48>
    15dc:	fff78793          	addi	a5,a5,-1
    15e0:	06078063          	beqz	a5,1640 <.L32>
    15e4:	01080813          	addi	a6,a6,16

00000000000015e8 <.L33>:
    15e8:	00083e83          	ld	t4,0(a6)
    15ec:	01e89fb3          	sll	t6,a7,t5
    15f0:	01c6d6b3          	srl	a3,a3,t3
    15f4:	01f6e6b3          	or	a3,a3,t6
    15f8:	00d73023          	sd	a3,0(a4)
    15fc:	00088693          	mv	a3,a7
    1600:	000e8893          	mv	a7,t4
    1604:	0800006f          	j	1684 <.L31>

0000000000001608 <.L25>:
    1608:	00068e93          	mv	t4,a3
    160c:	ff880813          	addi	a6,a6,-8
    1610:	fe830713          	addi	a4,t1,-24
    1614:	00278793          	addi	a5,a5,2
    1618:	00088693          	mv	a3,a7

000000000000161c <.L29>:
    161c:	01cedeb3          	srl	t4,t4,t3
    1620:	01e69fb3          	sll	t6,a3,t5
    1624:	01feeeb3          	or	t4,t4,t6
    1628:	01883883          	ld	a7,24(a6)
    162c:	ffc78793          	addi	a5,a5,-4
    1630:	01d73c23          	sd	t4,24(a4)
    1634:	02080813          	addi	a6,a6,32
    1638:	02070713          	addi	a4,a4,32
    163c:	fa0796e3          	bnez	a5,15e8 <.L33>

0000000000001640 <.L32>:
    1640:	01c6d6b3          	srl	a3,a3,t3
    1644:	01e898b3          	sll	a7,a7,t5
    1648:	0116e6b3          	or	a3,a3,a7
    164c:	00d73023          	sd	a3,0(a4)
    1650:	e7dff06f          	j	14cc <.L24>

0000000000001654 <.L26>:
    1654:	ff030713          	addi	a4,t1,-16
    1658:	00178793          	addi	a5,a5,1

000000000000165c <.L30>:
    165c:	01083e83          	ld	t4,16(a6)
    1660:	01c6d6b3          	srl	a3,a3,t3
    1664:	01e89fb3          	sll	t6,a7,t5
    1668:	01f6e6b3          	or	a3,a3,t6
    166c:	00d73823          	sd	a3,16(a4)
    1670:	000e8693          	mv	a3,t4
    1674:	00088e93          	mv	t4,a7
    1678:	fa5ff06f          	j	161c <.L29>

000000000000167c <.L48>:
    167c:	000f8813          	mv	a6,t6
    1680:	ff830713          	addi	a4,t1,-8

0000000000001684 <.L31>:
    1684:	00883e83          	ld	t4,8(a6)
    1688:	01e89fb3          	sll	t6,a7,t5
    168c:	01c6d6b3          	srl	a3,a3,t3
    1690:	01f6e6b3          	or	a3,a3,t6
    1694:	00d73423          	sd	a3,8(a4)
    1698:	00088693          	mv	a3,a7
    169c:	000e8893          	mv	a7,t4
    16a0:	fbdff06f          	j	165c <.L30>

00000000000016a4 <.L35>:
    16a4:	00e785b3          	add	a1,a5,a4
    16a8:	0005c803          	lbu	a6,0(a1)
    16ac:	00e685b3          	add	a1,a3,a4
    16b0:	00170713          	addi	a4,a4,1
    16b4:	01058023          	sb	a6,0(a1)
    16b8:	e29ff06f          	j	14e0 <.L34>

00000000000016bc <__GI_strnlen>:
    16bc:	00050613          	mv	a2,a0
    16c0:	00058513          	mv	a0,a1
    16c4:	06058a63          	beqz	a1,1738 <.L2>
    16c8:	00b606b3          	add	a3,a2,a1
    16cc:	00c6f463          	bgeu	a3,a2,16d4 <.L3>
    16d0:	fff00693          	li	a3,-1

00000000000016d4 <.L3>:
    16d4:	00060793          	mv	a5,a2

00000000000016d8 <.L4>:
    16d8:	0077f713          	andi	a4,a5,7
    16dc:	04071463          	bnez	a4,1724 <.L7>
    16e0:	00078713          	mv	a4,a5
    16e4:	00005517          	auipc	a0,0x5
    16e8:	23c53503          	ld	a0,572(a0) # 6920 <.LC0>
    16ec:	00005817          	auipc	a6,0x5
    16f0:	23c83803          	ld	a6,572(a6) # 6928 <.LC1>

00000000000016f4 <.L24>:
    16f4:	02d77c63          	bgeu	a4,a3,172c <.L9>
    16f8:	00073583          	ld	a1,0(a4)
    16fc:	00870793          	addi	a5,a4,8
    1700:	00a585b3          	add	a1,a1,a0
    1704:	0105f5b3          	and	a1,a1,a6
    1708:	08058e63          	beqz	a1,17a4 <.L20>
    170c:	ff87c583          	lbu	a1,-8(a5)
    1710:	0a058063          	beqz	a1,17b0 <.L19>
    1714:	ff97c583          	lbu	a1,-7(a5)
    1718:	02059663          	bnez	a1,1744 <.L10>
    171c:	00170793          	addi	a5,a4,1
    1720:	00c0006f          	j	172c <.L9>

0000000000001724 <.L7>:
    1724:	0007c703          	lbu	a4,0(a5)
    1728:	00071a63          	bnez	a4,173c <.L5>

000000000000172c <.L9>:
    172c:	00f6f463          	bgeu	a3,a5,1734 <.L17>
    1730:	00068793          	mv	a5,a3

0000000000001734 <.L17>:
    1734:	40c78533          	sub	a0,a5,a2

0000000000001738 <.L2>:
    1738:	00008067          	ret

000000000000173c <.L5>:
    173c:	00178793          	addi	a5,a5,1
    1740:	f99ff06f          	j	16d8 <.L4>

0000000000001744 <.L10>:
    1744:	ffa7c583          	lbu	a1,-6(a5)
    1748:	00059663          	bnez	a1,1754 <.L11>
    174c:	00270793          	addi	a5,a4,2
    1750:	fddff06f          	j	172c <.L9>

0000000000001754 <.L11>:
    1754:	ffb7c583          	lbu	a1,-5(a5)
    1758:	00059663          	bnez	a1,1764 <.L12>
    175c:	00370793          	addi	a5,a4,3
    1760:	fcdff06f          	j	172c <.L9>

0000000000001764 <.L12>:
    1764:	ffc7c583          	lbu	a1,-4(a5)
    1768:	00059663          	bnez	a1,1774 <.L13>
    176c:	00470793          	addi	a5,a4,4
    1770:	fbdff06f          	j	172c <.L9>

0000000000001774 <.L13>:
    1774:	ffd7c583          	lbu	a1,-3(a5)
    1778:	00059663          	bnez	a1,1784 <.L14>
    177c:	00570793          	addi	a5,a4,5
    1780:	fadff06f          	j	172c <.L9>

0000000000001784 <.L14>:
    1784:	ffe7c583          	lbu	a1,-2(a5)
    1788:	00059663          	bnez	a1,1794 <.L15>
    178c:	00670793          	addi	a5,a4,6
    1790:	f9dff06f          	j	172c <.L9>

0000000000001794 <.L15>:
    1794:	fff7c583          	lbu	a1,-1(a5)
    1798:	00059663          	bnez	a1,17a4 <.L20>
    179c:	00770793          	addi	a5,a4,7
    17a0:	f8dff06f          	j	172c <.L9>

00000000000017a4 <.L20>:
    17a4:	00078713          	mv	a4,a5
    17a8:	00068793          	mv	a5,a3
    17ac:	f49ff06f          	j	16f4 <.L24>

00000000000017b0 <.L19>:
    17b0:	00070793          	mv	a5,a4
    17b4:	f79ff06f          	j	172c <.L9>

00000000000017b8 <__GI_memset>:
    17b8:	00700713          	li	a4,7
    17bc:	00050793          	mv	a5,a0
    17c0:	04c77863          	bgeu	a4,a2,1810 <.L2>
    17c4:	0ff5f813          	zext.b	a6,a1
    17c8:	00881713          	slli	a4,a6,0x8
    17cc:	01076733          	or	a4,a4,a6
    17d0:	01071693          	slli	a3,a4,0x10
    17d4:	00e6e6b3          	or	a3,a3,a4
    17d8:	02069713          	slli	a4,a3,0x20
    17dc:	00d76733          	or	a4,a4,a3
    17e0:	00c508b3          	add	a7,a0,a2

00000000000017e4 <.L3>:
    17e4:	0077f693          	andi	a3,a5,7
    17e8:	40f88633          	sub	a2,a7,a5
    17ec:	02069863          	bnez	a3,181c <.L4>
    17f0:	fc067813          	andi	a6,a2,-64
    17f4:	00f80833          	add	a6,a6,a5
    17f8:	00080693          	mv	a3,a6

00000000000017fc <.L5>:
    17fc:	03079663          	bne	a5,a6,1828 <.L6>
    1800:	03867813          	andi	a6,a2,56
    1804:	00f807b3          	add	a5,a6,a5

0000000000001808 <.L7>:
    1808:	04f69463          	bne	a3,a5,1850 <.L8>
    180c:	00767613          	andi	a2,a2,7

0000000000001810 <.L2>:
    1810:	00c78633          	add	a2,a5,a2

0000000000001814 <.L9>:
    1814:	04c79463          	bne	a5,a2,185c <.L10>
    1818:	00008067          	ret

000000000000181c <.L4>:
    181c:	01078023          	sb	a6,0(a5)
    1820:	00178793          	addi	a5,a5,1
    1824:	fc1ff06f          	j	17e4 <.L3>

0000000000001828 <.L6>:
    1828:	00e7b023          	sd	a4,0(a5)
    182c:	00e7b423          	sd	a4,8(a5)
    1830:	00e7b823          	sd	a4,16(a5)
    1834:	00e7bc23          	sd	a4,24(a5)
    1838:	02e7b023          	sd	a4,32(a5)
    183c:	02e7b423          	sd	a4,40(a5)
    1840:	02e7b823          	sd	a4,48(a5)
    1844:	02e7bc23          	sd	a4,56(a5)
    1848:	04078793          	addi	a5,a5,64
    184c:	fb1ff06f          	j	17fc <.L5>

0000000000001850 <.L8>:
    1850:	00e6b023          	sd	a4,0(a3)
    1854:	00868693          	addi	a3,a3,8
    1858:	fb1ff06f          	j	1808 <.L7>

000000000000185c <.L10>:
    185c:	00b78023          	sb	a1,0(a5)
    1860:	00178793          	addi	a5,a5,1
    1864:	fb1ff06f          	j	1814 <.L9>

0000000000001868 <__GI___glibc_strerror_r>:
    1868:	ff010113          	addi	sp,sp,-16
    186c:	00813023          	sd	s0,0(sp)
    1870:	00113423          	sd	ra,8(sp)
    1874:	00058413          	mv	s0,a1
    1878:	494010ef          	jal	ra,2d0c <__GI___xpg_strerror_r>
    187c:	00813083          	ld	ra,8(sp)
    1880:	00040513          	mv	a0,s0
    1884:	00013403          	ld	s0,0(sp)
    1888:	01010113          	addi	sp,sp,16
    188c:	00008067          	ret

0000000000001890 <__GI_isatty>:
    1890:	fb010113          	addi	sp,sp,-80
    1894:	00010593          	mv	a1,sp
    1898:	04113423          	sd	ra,72(sp)
    189c:	014000ef          	jal	ra,18b0 <__GI_tcgetattr>
    18a0:	04813083          	ld	ra,72(sp)
    18a4:	00153513          	seqz	a0,a0
    18a8:	05010113          	addi	sp,sp,80
    18ac:	00008067          	ret

00000000000018b0 <__GI_tcgetattr>:
    18b0:	fb010113          	addi	sp,sp,-80
    18b4:	04813023          	sd	s0,64(sp)
    18b8:	00058413          	mv	s0,a1
    18bc:	000055b7          	lui	a1,0x5
    18c0:	00810613          	addi	a2,sp,8
    18c4:	40158593          	addi	a1,a1,1025 # 5401 <.LVL27+0x1>
    18c8:	02913c23          	sd	s1,56(sp)
    18cc:	04113423          	sd	ra,72(sp)
    18d0:	234000ef          	jal	ra,1b04 <__GI_ioctl>
    18d4:	00050493          	mv	s1,a0
    18d8:	04051463          	bnez	a0,1920 <.L2>
    18dc:	00812783          	lw	a5,8(sp)
    18e0:	01300613          	li	a2,19
    18e4:	01910593          	addi	a1,sp,25
    18e8:	00f42023          	sw	a5,0(s0)
    18ec:	00c12783          	lw	a5,12(sp)
    18f0:	01140513          	addi	a0,s0,17
    18f4:	00f42223          	sw	a5,4(s0)
    18f8:	01012783          	lw	a5,16(sp)
    18fc:	00f42423          	sw	a5,8(s0)
    1900:	01412783          	lw	a5,20(sp)
    1904:	00f42623          	sw	a5,12(s0)
    1908:	01814783          	lbu	a5,24(sp)
    190c:	00f40823          	sb	a5,16(s0)
    1910:	2b0010ef          	jal	ra,2bc0 <__GI_mempcpy>
    1914:	00d00613          	li	a2,13
    1918:	00000593          	li	a1,0
    191c:	e9dff0ef          	jal	ra,17b8 <__GI_memset>

0000000000001920 <.L2>:
    1920:	04813083          	ld	ra,72(sp)
    1924:	04013403          	ld	s0,64(sp)
    1928:	00048513          	mv	a0,s1
    192c:	03813483          	ld	s1,56(sp)
    1930:	05010113          	addi	sp,sp,80
    1934:	00008067          	ret

0000000000001938 <__uClibc_init>:
    1938:	00007797          	auipc	a5,0x7
    193c:	20078793          	addi	a5,a5,512 # 8b38 <__pagesize>
    1940:	0007b703          	ld	a4,0(a5)
    1944:	02071063          	bnez	a4,1964 <.L1>
    1948:	00002737          	lui	a4,0x2
    194c:	00e7b023          	sd	a4,0(a5)
    1950:	00004717          	auipc	a4,0x4
    1954:	d4873703          	ld	a4,-696(a4) # 5698 <.LLST14+0x4c>
    1958:	00070793          	mv	a5,a4
    195c:	00078463          	beqz	a5,1964 <.L1>
    1960:	00070067          	jr	a4

0000000000001964 <.L1>:
    1964:	00008067          	ret

0000000000001968 <__uClibc_fini>:
    1968:	00007797          	auipc	a5,0x7
    196c:	1d87b783          	ld	a5,472(a5) # 8b40 <__rtld_fini>
    1970:	00078463          	beqz	a5,1978 <.L8>
    1974:	00078067          	jr	a5

0000000000001978 <.L8>:
    1978:	00008067          	ret

000000000000197c <__uClibc_main>:
    197c:	ed010113          	addi	sp,sp,-304
    1980:	00004717          	auipc	a4,0x4
    1984:	ca073703          	ld	a4,-864(a4) # 5620 <_GLOBAL_OFFSET_TABLE_+0x10>
    1988:	12813023          	sd	s0,288(sp)
    198c:	11213823          	sd	s2,272(sp)
    1990:	11313423          	sd	s3,264(sp)
    1994:	11413023          	sd	s4,256(sp)
    1998:	12113423          	sd	ra,296(sp)
    199c:	10913c23          	sd	s1,280(sp)
    19a0:	0f513c23          	sd	s5,248(sp)
    19a4:	0f613823          	sd	s6,240(sp)
    19a8:	01073023          	sd	a6,0(a4)
    19ac:	00007717          	auipc	a4,0x7
    19b0:	18f73a23          	sd	a5,404(a4) # 8b40 <__rtld_fini>
    19b4:	00158793          	addi	a5,a1,1
    19b8:	00379793          	slli	a5,a5,0x3
    19bc:	00063683          	ld	a3,0(a2)
    19c0:	00f60733          	add	a4,a2,a5
    19c4:	00004a17          	auipc	s4,0x4
    19c8:	c6ca3a03          	ld	s4,-916(s4) # 5630 <_GLOBAL_OFFSET_TABLE_+0x20>
    19cc:	00ea3023          	sd	a4,0(s4)
    19d0:	00050993          	mv	s3,a0
    19d4:	00058913          	mv	s2,a1
    19d8:	00060413          	mv	s0,a2
    19dc:	00d71863          	bne	a4,a3,19ec <.L11>
    19e0:	ff878793          	addi	a5,a5,-8
    19e4:	00f607b3          	add	a5,a2,a5
    19e8:	00fa3023          	sd	a5,0(s4)

00000000000019ec <.L11>:
    19ec:	00010a93          	mv	s5,sp
    19f0:	0f000613          	li	a2,240
    19f4:	00000593          	li	a1,0
    19f8:	000a8513          	mv	a0,s5
    19fc:	dbdff0ef          	jal	ra,17b8 <__GI_memset>
    1a00:	000a3483          	ld	s1,0(s4)

0000000000001a04 <.L12>:
    1a04:	0004b783          	ld	a5,0(s1)
    1a08:	00848493          	addi	s1,s1,8
    1a0c:	fe079ce3          	bnez	a5,1a04 <.L12>
    1a10:	00e00b13          	li	s6,14

0000000000001a14 <.L13>:
    1a14:	0004b783          	ld	a5,0(s1)
    1a18:	08079863          	bnez	a5,1aa8 <.L15>
    1a1c:	000a8513          	mv	a0,s5
    1a20:	0a8000ef          	jal	ra,1ac8 <_dl_aux_init>
    1a24:	f15ff0ef          	jal	ra,1938 <__uClibc_init>
    1a28:	00043783          	ld	a5,0(s0)
    1a2c:	00005717          	auipc	a4,0x5
    1a30:	06f73e23          	sd	a5,124(a4) # 6aa8 <__uclibc_progname>
    1a34:	02078c63          	beqz	a5,1a6c <.L16>
    1a38:	00004717          	auipc	a4,0x4
    1a3c:	c7873703          	ld	a4,-904(a4) # 56b0 <.LLST14+0x64>
    1a40:	00f73023          	sd	a5,0(a4)
    1a44:	00043503          	ld	a0,0(s0)
    1a48:	02f00593          	li	a1,47
    1a4c:	108010ef          	jal	ra,2b54 <__GI_strrchr>
    1a50:	00004717          	auipc	a4,0x4
    1a54:	c0873703          	ld	a4,-1016(a4) # 5658 <.LLST14+0xc>
    1a58:	00a73023          	sd	a0,0(a4)
    1a5c:	00150793          	addi	a5,a0,1
    1a60:	00051463          	bnez	a0,1a68 <.L31>
    1a64:	00043783          	ld	a5,0(s0)

0000000000001a68 <.L31>:
    1a68:	00f73023          	sd	a5,0(a4)

0000000000001a6c <.L16>:
    1a6c:	00004797          	auipc	a5,0x4
    1a70:	c247b783          	ld	a5,-988(a5) # 5690 <.LLST14+0x44>
    1a74:	00078663          	beqz	a5,1a80 <.L18>
    1a78:	0e0000ef          	jal	ra,1b58 <__errno_location>
    1a7c:	00052023          	sw	zero,0(a0)

0000000000001a80 <.L18>:
    1a80:	00004797          	auipc	a5,0x4
    1a84:	bf07b783          	ld	a5,-1040(a5) # 5670 <.LLST14+0x24>
    1a88:	00078663          	beqz	a5,1a94 <.L19>
    1a8c:	0d8000ef          	jal	ra,1b64 <__h_errno_location>
    1a90:	00052023          	sw	zero,0(a0)

0000000000001a94 <.L19>:
    1a94:	000a3603          	ld	a2,0(s4)
    1a98:	00040593          	mv	a1,s0
    1a9c:	00090513          	mv	a0,s2
    1aa0:	000980e7          	jalr	s3
    1aa4:	368010ef          	jal	ra,2e0c <__GI_exit>

0000000000001aa8 <.L15>:
    1aa8:	00fb6c63          	bltu	s6,a5,1ac0 <.L14>
    1aac:	00479513          	slli	a0,a5,0x4
    1ab0:	01000613          	li	a2,16
    1ab4:	00048593          	mv	a1,s1
    1ab8:	00aa8533          	add	a0,s5,a0
    1abc:	961ff0ef          	jal	ra,141c <__GI_memcpy>

0000000000001ac0 <.L14>:
    1ac0:	01048493          	addi	s1,s1,16
    1ac4:	f51ff06f          	j	1a14 <.L13>

0000000000001ac8 <_dl_aux_init>:
    1ac8:	03853703          	ld	a4,56(a0)
    1acc:	00004797          	auipc	a5,0x4
    1ad0:	b947b783          	ld	a5,-1132(a5) # 5660 <.LLST14+0x14>
    1ad4:	00e7b023          	sd	a4,0(a5)
    1ad8:	05853703          	ld	a4,88(a0)
    1adc:	00004797          	auipc	a5,0x4
    1ae0:	bac7b783          	ld	a5,-1108(a5) # 5688 <.LLST14+0x3c>
    1ae4:	00e7b023          	sd	a4,0(a5)
    1ae8:	06853783          	ld	a5,104(a0)
    1aec:	00079463          	bnez	a5,1af4 <.L2>
    1af0:	000027b7          	lui	a5,0x2

0000000000001af4 <.L2>:
    1af4:	00004717          	auipc	a4,0x4
    1af8:	bb473703          	ld	a4,-1100(a4) # 56a8 <.LLST14+0x5c>
    1afc:	00f73023          	sd	a5,0(a4)
    1b00:	00008067          	ret

0000000000001b04 <__GI_ioctl>:
    1b04:	fc010113          	addi	sp,sp,-64
    1b08:	02f13423          	sd	a5,40(sp)
    1b0c:	01010793          	addi	a5,sp,16
    1b10:	03113c23          	sd	a7,56(sp)
    1b14:	00c13823          	sd	a2,16(sp)
    1b18:	00d13c23          	sd	a3,24(sp)
    1b1c:	02e13023          	sd	a4,32(sp)
    1b20:	03013823          	sd	a6,48(sp)
    1b24:	00f13423          	sd	a5,8(sp)
    1b28:	01d00893          	li	a7,29
    1b2c:	00000073          	ecall
    1b30:	fffff7b7          	lui	a5,0xfffff
    1b34:	00a7fc63          	bgeu	a5,a0,1b4c <.L2>
    1b38:	40a0053b          	negw	a0,a0
    1b3c:	00004797          	auipc	a5,0x4
    1b40:	aec7b783          	ld	a5,-1300(a5) # 5628 <_GLOBAL_OFFSET_TABLE_+0x18>
    1b44:	00a7a023          	sw	a0,0(a5)
    1b48:	fff00513          	li	a0,-1

0000000000001b4c <.L2>:
    1b4c:	0005051b          	sext.w	a0,a0
    1b50:	04010113          	addi	sp,sp,64
    1b54:	00008067          	ret

0000000000001b58 <__errno_location>:
    1b58:	00004517          	auipc	a0,0x4
    1b5c:	ad053503          	ld	a0,-1328(a0) # 5628 <_GLOBAL_OFFSET_TABLE_+0x18>
    1b60:	00008067          	ret

0000000000001b64 <__h_errno_location>:
    1b64:	00004517          	auipc	a0,0x4
    1b68:	ad453503          	ld	a0,-1324(a0) # 5638 <_GLOBAL_OFFSET_TABLE_+0x28>
    1b6c:	00008067          	ret

0000000000001b70 <__stdio_WRITE>:
    1b70:	fb010113          	addi	sp,sp,-80
    1b74:	03413023          	sd	s4,32(sp)
    1b78:	fff00a13          	li	s4,-1
    1b7c:	04813023          	sd	s0,64(sp)
    1b80:	02913c23          	sd	s1,56(sp)
    1b84:	03213823          	sd	s2,48(sp)
    1b88:	03313423          	sd	s3,40(sp)
    1b8c:	01513c23          	sd	s5,24(sp)
    1b90:	01613823          	sd	s6,16(sp)
    1b94:	01713423          	sd	s7,8(sp)
    1b98:	04113423          	sd	ra,72(sp)
    1b9c:	00050493          	mv	s1,a0
    1ba0:	00058413          	mv	s0,a1
    1ba4:	00060993          	mv	s3,a2
    1ba8:	00060913          	mv	s2,a2
    1bac:	001a5a13          	srli	s4,s4,0x1
    1bb0:	ffe00b13          	li	s6,-2
    1bb4:	00004a97          	auipc	s5,0x4
    1bb8:	a74aba83          	ld	s5,-1420(s5) # 5628 <_GLOBAL_OFFSET_TABLE_+0x18>
    1bbc:	00b00b93          	li	s7,11

0000000000001bc0 <.L2>:
    1bc0:	04090a63          	beqz	s2,1c14 <.L10>
    1bc4:	000a0613          	mv	a2,s4
    1bc8:	00094463          	bltz	s2,1bd0 <.L3>
    1bcc:	00090613          	mv	a2,s2

0000000000001bd0 <.L3>:
    1bd0:	0044a503          	lw	a0,4(s1)
    1bd4:	09651a63          	bne	a0,s6,1c68 <.L4>
    1bd8:	0504b783          	ld	a5,80(s1)
    1bdc:	06079463          	bnez	a5,1c44 <.L5>
    1be0:	01600793          	li	a5,22
    1be4:	00faa023          	sw	a5,0(s5)

0000000000001be8 <.L6>:
    1be8:	0004d783          	lhu	a5,0(s1)
    1bec:	00400713          	li	a4,4
    1bf0:	0087e793          	ori	a5,a5,8
    1bf4:	00f49023          	sh	a5,0(s1)
    1bf8:	00004797          	auipc	a5,0x4
    1bfc:	a307b783          	ld	a5,-1488(a5) # 5628 <_GLOBAL_OFFSET_TABLE_+0x18>
    1c00:	0007a783          	lw	a5,0(a5)
    1c04:	06e78863          	beq	a5,a4,1c74 <.L9>
    1c08:	00b00713          	li	a4,11
    1c0c:	06e78463          	beq	a5,a4,1c74 <.L9>

0000000000001c10 <.L11>:
    1c10:	412989b3          	sub	s3,s3,s2

0000000000001c14 <.L10>:
    1c14:	04813083          	ld	ra,72(sp)
    1c18:	04013403          	ld	s0,64(sp)
    1c1c:	03813483          	ld	s1,56(sp)
    1c20:	03013903          	ld	s2,48(sp)
    1c24:	02013a03          	ld	s4,32(sp)
    1c28:	01813a83          	ld	s5,24(sp)
    1c2c:	01013b03          	ld	s6,16(sp)
    1c30:	00813b83          	ld	s7,8(sp)
    1c34:	00098513          	mv	a0,s3
    1c38:	02813983          	ld	s3,40(sp)
    1c3c:	05010113          	addi	sp,sp,80
    1c40:	00008067          	ret

0000000000001c44 <.L5>:
    1c44:	0404b503          	ld	a0,64(s1)
    1c48:	017aa023          	sw	s7,0(s5)
    1c4c:	00040593          	mv	a1,s0
    1c50:	000780e7          	jalr	a5
    1c54:	f8050ae3          	beqz	a0,1be8 <.L6>

0000000000001c58 <.L7>:
    1c58:	f80548e3          	bltz	a0,1be8 <.L6>
    1c5c:	40a90933          	sub	s2,s2,a0
    1c60:	00a40433          	add	s0,s0,a0
    1c64:	f5dff06f          	j	1bc0 <.L2>

0000000000001c68 <.L4>:
    1c68:	00040593          	mv	a1,s0
    1c6c:	1e4010ef          	jal	ra,2e50 <__GI_write>
    1c70:	fe9ff06f          	j	1c58 <.L7>

0000000000001c74 <.L9>:
    1c74:	0084b783          	ld	a5,8(s1)
    1c78:	0104b683          	ld	a3,16(s1)
    1c7c:	40f68733          	sub	a4,a3,a5
    1c80:	f8f688e3          	beq	a3,a5,1c10 <.L11>
    1c84:	00e97463          	bgeu	s2,a4,1c8c <.L12>
    1c88:	00090713          	mv	a4,s2

0000000000001c8c <.L12>:
    1c8c:	00a00613          	li	a2,10
    1c90:	00e40733          	add	a4,s0,a4

0000000000001c94 <.L15>:
    1c94:	00044683          	lbu	a3,0(s0)
    1c98:	00d78023          	sb	a3,0(a5)
    1c9c:	00c69863          	bne	a3,a2,1cac <.L13>
    1ca0:	0004d683          	lhu	a3,0(s1)
    1ca4:	1006f693          	andi	a3,a3,256
    1ca8:	00069863          	bnez	a3,1cb8 <.L14>

0000000000001cac <.L13>:
    1cac:	00140413          	addi	s0,s0,1
    1cb0:	00178793          	addi	a5,a5,1
    1cb4:	fee410e3          	bne	s0,a4,1c94 <.L15>

0000000000001cb8 <.L14>:
    1cb8:	0084b703          	ld	a4,8(s1)
    1cbc:	00f4bc23          	sd	a5,24(s1)
    1cc0:	40e787b3          	sub	a5,a5,a4
    1cc4:	40f90933          	sub	s2,s2,a5
    1cc8:	f49ff06f          	j	1c10 <.L11>

0000000000001ccc <__stdio_fwrite>:
    1ccc:	00065783          	lhu	a5,0(a2)
    1cd0:	fd010113          	addi	sp,sp,-48
    1cd4:	02813023          	sd	s0,32(sp)
    1cd8:	00913c23          	sd	s1,24(sp)
    1cdc:	01213823          	sd	s2,16(sp)
    1ce0:	02113423          	sd	ra,40(sp)
    1ce4:	01313423          	sd	s3,8(sp)
    1ce8:	2007f793          	andi	a5,a5,512
    1cec:	00050913          	mv	s2,a0
    1cf0:	00058413          	mv	s0,a1
    1cf4:	00060493          	mv	s1,a2
    1cf8:	02078663          	beqz	a5,1d24 <.L2>

0000000000001cfc <.L10>:
    1cfc:	00040613          	mv	a2,s0
    1d00:	02013403          	ld	s0,32(sp)
    1d04:	02813083          	ld	ra,40(sp)
    1d08:	00813983          	ld	s3,8(sp)
    1d0c:	00090593          	mv	a1,s2
    1d10:	00048513          	mv	a0,s1
    1d14:	01013903          	ld	s2,16(sp)
    1d18:	01813483          	ld	s1,24(sp)
    1d1c:	03010113          	addi	sp,sp,48
    1d20:	e51ff06f          	j	1b70 <__stdio_WRITE>

0000000000001d24 <.L2>:
    1d24:	01863503          	ld	a0,24(a2)
    1d28:	01063783          	ld	a5,16(a2)
    1d2c:	00462683          	lw	a3,4(a2)
    1d30:	ffd00713          	li	a4,-3
    1d34:	40a787b3          	sub	a5,a5,a0
    1d38:	04e69463          	bne	a3,a4,1d80 <.L3>
    1d3c:	00058993          	mv	s3,a1
    1d40:	00b7f463          	bgeu	a5,a1,1d48 <.L4>
    1d44:	00078993          	mv	s3,a5

0000000000001d48 <.L4>:
    1d48:	00098613          	mv	a2,s3
    1d4c:	00090593          	mv	a1,s2
    1d50:	eccff0ef          	jal	ra,141c <__GI_memcpy>
    1d54:	0184b783          	ld	a5,24(s1)
    1d58:	013787b3          	add	a5,a5,s3

0000000000001d5c <.L24>:
    1d5c:	00f4bc23          	sd	a5,24(s1)

0000000000001d60 <.L5>:
    1d60:	02813083          	ld	ra,40(sp)
    1d64:	00040513          	mv	a0,s0
    1d68:	02013403          	ld	s0,32(sp)
    1d6c:	01813483          	ld	s1,24(sp)
    1d70:	01013903          	ld	s2,16(sp)
    1d74:	00813983          	ld	s3,8(sp)
    1d78:	03010113          	addi	sp,sp,48
    1d7c:	00008067          	ret

0000000000001d80 <.L3>:
    1d80:	08b7e063          	bltu	a5,a1,1e00 <.L6>
    1d84:	00058613          	mv	a2,a1
    1d88:	00090593          	mv	a1,s2
    1d8c:	e90ff0ef          	jal	ra,141c <__GI_memcpy>
    1d90:	0184b783          	ld	a5,24(s1)
    1d94:	008787b3          	add	a5,a5,s0
    1d98:	00f4bc23          	sd	a5,24(s1)
    1d9c:	0004d783          	lhu	a5,0(s1)
    1da0:	1007f793          	andi	a5,a5,256
    1da4:	fa078ee3          	beqz	a5,1d60 <.L5>
    1da8:	00040613          	mv	a2,s0
    1dac:	00a00593          	li	a1,10
    1db0:	00090513          	mv	a0,s2
    1db4:	485000ef          	jal	ra,2a38 <__GI_memrchr>
    1db8:	fa0504e3          	beqz	a0,1d60 <.L5>
    1dbc:	00048513          	mv	a0,s1
    1dc0:	c8cfe0ef          	jal	ra,24c <__stdio_wcommit>
    1dc4:	f8050ee3          	beqz	a0,1d60 <.L5>
    1dc8:	00040613          	mv	a2,s0
    1dcc:	00857463          	bgeu	a0,s0,1dd4 <.L9>
    1dd0:	00050613          	mv	a2,a0

0000000000001dd4 <.L9>:
    1dd4:	40c40533          	sub	a0,s0,a2
    1dd8:	00a00593          	li	a1,10
    1ddc:	00a90533          	add	a0,s2,a0
    1de0:	345000ef          	jal	ra,2924 <__GI_memchr>
    1de4:	f6050ee3          	beqz	a0,1d60 <.L5>
    1de8:	0184b783          	ld	a5,24(s1)
    1dec:	00890933          	add	s2,s2,s0
    1df0:	40a90533          	sub	a0,s2,a0
    1df4:	40a40433          	sub	s0,s0,a0
    1df8:	40a787b3          	sub	a5,a5,a0
    1dfc:	f61ff06f          	j	1d5c <.L24>

0000000000001e00 <.L6>:
    1e00:	00863783          	ld	a5,8(a2)
    1e04:	eea78ce3          	beq	a5,a0,1cfc <.L10>
    1e08:	00060513          	mv	a0,a2
    1e0c:	c40fe0ef          	jal	ra,24c <__stdio_wcommit>
    1e10:	ee0506e3          	beqz	a0,1cfc <.L10>
    1e14:	00000413          	li	s0,0
    1e18:	f49ff06f          	j	1d60 <.L5>

0000000000001e1c <__stdio_trans2w>:
    1e1c:	00055603          	lhu	a2,0(a0)
    1e20:	ff010113          	addi	sp,sp,-16
    1e24:	00813023          	sd	s0,0(sp)
    1e28:	00113423          	sd	ra,8(sp)
    1e2c:	02067793          	andi	a5,a2,32
    1e30:	00050413          	mv	s0,a0
    1e34:	02078a63          	beqz	a5,1e68 <.L2>
    1e38:	00003797          	auipc	a5,0x3
    1e3c:	7f07b783          	ld	a5,2032(a5) # 5628 <_GLOBAL_OFFSET_TABLE_+0x18>
    1e40:	00900713          	li	a4,9
    1e44:	00e7a023          	sw	a4,0(a5)

0000000000001e48 <.L3>:
    1e48:	00045783          	lhu	a5,0(s0)
    1e4c:	fff00513          	li	a0,-1
    1e50:	0087e793          	ori	a5,a5,8
    1e54:	00f41023          	sh	a5,0(s0)

0000000000001e58 <.L4>:
    1e58:	00813083          	ld	ra,8(sp)
    1e5c:	00013403          	ld	s0,0(sp)
    1e60:	01010113          	addi	sp,sp,16
    1e64:	00008067          	ret

0000000000001e68 <.L2>:
    1e68:	00367793          	andi	a5,a2,3
    1e6c:	02078463          	beqz	a5,1e94 <.L5>
    1e70:	00467793          	andi	a5,a2,4
    1e74:	04078263          	beqz	a5,1eb8 <.L6>

0000000000001e78 <.L8>:
    1e78:	00045783          	lhu	a5,0(s0)
    1e7c:	ffc7f793          	andi	a5,a5,-4
    1e80:	00f41023          	sh	a5,0(s0)
    1e84:	00843783          	ld	a5,8(s0)
    1e88:	02f43423          	sd	a5,40(s0)
    1e8c:	00f43c23          	sd	a5,24(s0)
    1e90:	02f43023          	sd	a5,32(s0)

0000000000001e94 <.L5>:
    1e94:	00045783          	lhu	a5,0(s0)
    1e98:	00000513          	li	a0,0
    1e9c:	0407e713          	ori	a4,a5,64
    1ea0:	00e41023          	sh	a4,0(s0)
    1ea4:	3007f793          	andi	a5,a5,768
    1ea8:	fa0798e3          	bnez	a5,1e58 <.L4>
    1eac:	01043783          	ld	a5,16(s0)
    1eb0:	02f43823          	sd	a5,48(s0)
    1eb4:	fa5ff06f          	j	1e58 <.L4>

0000000000001eb8 <.L6>:
    1eb8:	02053703          	ld	a4,32(a0)
    1ebc:	01853783          	ld	a5,24(a0)
    1ec0:	00f71663          	bne	a4,a5,1ecc <.L7>
    1ec4:	00267793          	andi	a5,a2,2
    1ec8:	fa0788e3          	beqz	a5,1e78 <.L8>

0000000000001ecc <.L7>:
    1ecc:	40067613          	andi	a2,a2,1024
    1ed0:	00c03633          	snez	a2,a2
    1ed4:	00160613          	addi	a2,a2,1
    1ed8:	00000593          	li	a1,0
    1edc:	00040513          	mv	a0,s0
    1ee0:	7c5000ef          	jal	ra,2ea4 <__GI_fseek>
    1ee4:	f8050ae3          	beqz	a0,1e78 <.L8>
    1ee8:	f61ff06f          	j	1e48 <.L3>

0000000000001eec <_load_inttype>:
    1eec:	000017b7          	lui	a5,0x1
    1ef0:	c0078793          	addi	a5,a5,-1024 # c00 <.L19+0x1c>
    1ef4:	00f577b3          	and	a5,a0,a5
    1ef8:	04064063          	bltz	a2,1f38 <.L2>
    1efc:	00078663          	beqz	a5,1f08 <.L3>

0000000000001f00 <.L11>:
    1f00:	0005b503          	ld	a0,0(a1)
    1f04:	00008067          	ret

0000000000001f08 <.L3>:
    1f08:	10000713          	li	a4,256
    1f0c:	0005a783          	lw	a5,0(a1)
    1f10:	00e51a63          	bne	a0,a4,1f24 <.L6>
    1f14:	0ff7f793          	zext.b	a5,a5

0000000000001f18 <.L7>:
    1f18:	02079513          	slli	a0,a5,0x20
    1f1c:	02055513          	srli	a0,a0,0x20
    1f20:	00008067          	ret

0000000000001f24 <.L6>:
    1f24:	20000713          	li	a4,512
    1f28:	fee518e3          	bne	a0,a4,1f18 <.L7>
    1f2c:	03079793          	slli	a5,a5,0x30
    1f30:	0307d793          	srli	a5,a5,0x30
    1f34:	fe5ff06f          	j	1f18 <.L7>

0000000000001f38 <.L2>:
    1f38:	fc0794e3          	bnez	a5,1f00 <.L11>
    1f3c:	10000713          	li	a4,256
    1f40:	0005a783          	lw	a5,0(a1)
    1f44:	00e51a63          	bne	a0,a4,1f58 <.L9>
    1f48:	0187979b          	slliw	a5,a5,0x18
    1f4c:	4187d79b          	sraiw	a5,a5,0x18

0000000000001f50 <.L10>:
    1f50:	00078513          	mv	a0,a5
    1f54:	00008067          	ret

0000000000001f58 <.L9>:
    1f58:	20000713          	li	a4,512
    1f5c:	fee51ae3          	bne	a0,a4,1f50 <.L10>
    1f60:	0107979b          	slliw	a5,a5,0x10
    1f64:	4107d79b          	sraiw	a5,a5,0x10
    1f68:	fe9ff06f          	j	1f50 <.L10>

0000000000001f6c <_store_inttype>:
    1f6c:	10000793          	li	a5,256
    1f70:	00f59663          	bne	a1,a5,1f7c <.L2>
    1f74:	00c50023          	sb	a2,0(a0)
    1f78:	00008067          	ret

0000000000001f7c <.L2>:
    1f7c:	8005879b          	addiw	a5,a1,-2048
    1f80:	00079663          	bnez	a5,1f8c <.L4>

0000000000001f84 <.L7>:
    1f84:	00c53023          	sd	a2,0(a0)
    1f88:	00008067          	ret

0000000000001f8c <.L4>:
    1f8c:	20000793          	li	a5,512
    1f90:	00f59663          	bne	a1,a5,1f9c <.L5>
    1f94:	00c51023          	sh	a2,0(a0)
    1f98:	00008067          	ret

0000000000001f9c <.L5>:
    1f9c:	40000793          	li	a5,1024
    1fa0:	fef582e3          	beq	a1,a5,1f84 <.L7>
    1fa4:	00c52023          	sw	a2,0(a0)
    1fa8:	00008067          	ret

0000000000001fac <_uintmaxtostr>:
    1fac:	00050893          	mv	a7,a0
    1fb0:	00000e13          	li	t3,0
    1fb4:	00065a63          	bgez	a2,1fc8 <.L2>
    1fb8:	40c0063b          	negw	a2,a2
    1fbc:	0005d663          	bgez	a1,1fc8 <.L2>
    1fc0:	40b005b3          	neg	a1,a1
    1fc4:	00100e13          	li	t3,1

0000000000001fc8 <.L2>:
    1fc8:	fff00813          	li	a6,-1
    1fcc:	02c85ebb          	divuw	t4,a6,a2
    1fd0:	00088023          	sb	zero,0(a7)
    1fd4:	0006031b          	sext.w	t1,a2
    1fd8:	02c8783b          	remuw	a6,a6,a2
    1fdc:	0018081b          	addiw	a6,a6,1
    1fe0:	01061663          	bne	a2,a6,1fec <.L3>
    1fe4:	001e8e9b          	addiw	t4,t4,1
    1fe8:	00000813          	li	a6,0

0000000000001fec <.L3>:
    1fec:	0005879b          	sext.w	a5,a1
    1ff0:	00900f13          	li	t5,9
    1ff4:	4205d593          	srai	a1,a1,0x20

0000000000001ff8 <.L7>:
    1ff8:	0267f63b          	remuw	a2,a5,t1
    1ffc:	0267d7bb          	divuw	a5,a5,t1
    2000:	00060513          	mv	a0,a2
    2004:	00078713          	mv	a4,a5
    2008:	02058463          	beqz	a1,2030 <.L4>
    200c:	0265f7bb          	remuw	a5,a1,t1
    2010:	02f8063b          	mulw	a2,a6,a5
    2014:	02fe87bb          	mulw	a5,t4,a5
    2018:	00a6063b          	addw	a2,a2,a0
    201c:	00e787bb          	addw	a5,a5,a4
    2020:	0265d5bb          	divuw	a1,a1,t1
    2024:	0266573b          	divuw	a4,a2,t1
    2028:	0266763b          	remuw	a2,a2,t1
    202c:	00e787bb          	addw	a5,a5,a4

0000000000002030 <.L4>:
    2030:	0ff67713          	zext.b	a4,a2
    2034:	02cf6c63          	bltu	t5,a2,206c <.L5>
    2038:	0307071b          	addiw	a4,a4,48

000000000000203c <.L18>:
    203c:	0ff77713          	zext.b	a4,a4
    2040:	fee88fa3          	sb	a4,-1(a7)
    2044:	00f5e733          	or	a4,a1,a5
    2048:	fff88513          	addi	a0,a7,-1
    204c:	00071c63          	bnez	a4,2064 <.L11>
    2050:	000e0863          	beqz	t3,2060 <.L1>
    2054:	02d00793          	li	a5,45
    2058:	fef50fa3          	sb	a5,-1(a0)
    205c:	ffe88513          	addi	a0,a7,-2

0000000000002060 <.L1>:
    2060:	00008067          	ret

0000000000002064 <.L11>:
    2064:	00050893          	mv	a7,a0
    2068:	f91ff06f          	j	1ff8 <.L7>

000000000000206c <.L5>:
    206c:	00e6873b          	addw	a4,a3,a4
    2070:	fcdff06f          	j	203c <.L18>

0000000000002074 <_fpmaxtostr>:
    2074:	e3010113          	addi	sp,sp,-464
    2078:	0046a783          	lw	a5,4(a3)
    207c:	1b213823          	sd	s2,432(sp)
    2080:	0086c903          	lbu	s2,8(a3)
    2084:	02f13023          	sd	a5,32(sp)
    2088:	06500793          	li	a5,101
    208c:	1c813023          	sd	s0,448(sp)
    2090:	1a913c23          	sd	s1,440(sp)
    2094:	1b313423          	sd	s3,424(sp)
    2098:	1b413023          	sd	s4,416(sp)
    209c:	19713423          	sd	s7,392(sp)
    20a0:	06f10423          	sb	a5,104(sp)
    20a4:	1c113423          	sd	ra,456(sp)
    20a8:	19513c23          	sd	s5,408(sp)
    20ac:	19613823          	sd	s6,400(sp)
    20b0:	19813023          	sd	s8,384(sp)
    20b4:	17913c23          	sd	s9,376(sp)
    20b8:	17a13823          	sd	s10,368(sp)
    20bc:	17b13423          	sd	s11,360(sp)
    20c0:	04a13023          	sd	a0,64(sp)
    20c4:	04e13423          	sd	a4,72(sp)
    20c8:	02096493          	ori	s1,s2,32
    20cc:	06100793          	li	a5,97
    20d0:	0006a983          	lw	s3,0(a3)
    20d4:	00058413          	mv	s0,a1
    20d8:	00060b93          	mv	s7,a2
    20dc:	00068a13          	mv	s4,a3
    20e0:	1cf49263          	bne	s1,a5,22a4 <.L2>
    20e4:	07000793          	li	a5,112
    20e8:	06f10423          	sb	a5,104(sp)
    20ec:	00013c23          	sd	zero,24(sp)
    20f0:	0009d863          	bgez	s3,2100 <.L3>
    20f4:	00100793          	li	a5,1
    20f8:	00f13c23          	sd	a5,24(sp)
    20fc:	01d00993          	li	s3,29

0000000000002100 <.L3>:
    2100:	00ca2783          	lw	a5,12(s4)
    2104:	06010023          	sb	zero,96(sp)
    2108:	0027f713          	andi	a4,a5,2
    210c:	1a070463          	beqz	a4,22b4 <.L4>
    2110:	02b00793          	li	a5,43

0000000000002114 <.L116>:
    2114:	06f10023          	sb	a5,96(sp)

0000000000002118 <.L5>:
    2118:	00040613          	mv	a2,s0
    211c:	000b8693          	mv	a3,s7
    2120:	00040513          	mv	a0,s0
    2124:	000b8593          	mv	a1,s7
    2128:	060100a3          	sb	zero,97(sp)
    212c:	0c013423          	sd	zero,200(sp)
    2130:	049010ef          	jal	ra,3978 <__eqtf2>
    2134:	18050863          	beqz	a0,22c4 <.L110>
    2138:	00800793          	li	a5,8
    213c:	0cf13423          	sd	a5,200(sp)

0000000000002140 <.L8>:
    2140:	02000793          	li	a5,32
    2144:	00fa2823          	sw	a5,16(s4)
    2148:	07000793          	li	a5,112
    214c:	0af13c23          	sd	a5,184(sp)
    2150:	00300793          	li	a5,3
    2154:	0cf13023          	sd	a5,192(sp)
    2158:	06000793          	li	a5,96
    215c:	0127e863          	bltu	a5,s2,216c <.L16>
    2160:	0c813783          	ld	a5,200(sp)
    2164:	00478793          	addi	a5,a5,4
    2168:	0cf13423          	sd	a5,200(sp)

000000000000216c <.L16>:
    216c:	0c813783          	ld	a5,200(sp)
    2170:	d4018713          	addi	a4,gp,-704 # 5c20 <fmt>
    2174:	00090693          	mv	a3,s2
    2178:	00e787b3          	add	a5,a5,a4
    217c:	0cf13423          	sd	a5,200(sp)
    2180:	0d010413          	addi	s0,sp,208

0000000000002184 <.L17>:
    2184:	0c010793          	addi	a5,sp,192

0000000000002188 <.L56>:
    2188:	0007b703          	ld	a4,0(a5)
    218c:	02013603          	ld	a2,32(sp)
    2190:	01878793          	addi	a5,a5,24
    2194:	40e6073b          	subw	a4,a2,a4
    2198:	02e13023          	sd	a4,32(sp)
    219c:	fe87e6e3          	bltu	a5,s0,2188 <.L56>
    21a0:	06014703          	lbu	a4,96(sp)
    21a4:	0a000793          	li	a5,160
    21a8:	0af13023          	sd	a5,160(sp)
    21ac:	00e035b3          	snez	a1,a4
    21b0:	0206e793          	ori	a5,a3,32
    21b4:	06010613          	addi	a2,sp,96
    21b8:	0ab13423          	sd	a1,168(sp)
    21bc:	0ac13823          	sd	a2,176(sp)
    21c0:	0ff7f793          	zext.b	a5,a5
    21c4:	06100693          	li	a3,97
    21c8:	00058713          	mv	a4,a1
    21cc:	02d79a63          	bne	a5,a3,2200 <.L57>
    21d0:	0b813683          	ld	a3,184(sp)
    21d4:	00f00793          	li	a5,15
    21d8:	02d7d463          	bge	a5,a3,2200 <.L57>
    21dc:	00b607b3          	add	a5,a2,a1
    21e0:	03000693          	li	a3,48
    21e4:	00d78023          	sb	a3,0(a5)
    21e8:	06814683          	lbu	a3,104(sp)
    21ec:	00258713          	addi	a4,a1,2
    21f0:	00078123          	sb	zero,2(a5)
    21f4:	0086869b          	addiw	a3,a3,8
    21f8:	00d780a3          	sb	a3,1(a5)
    21fc:	0ae13423          	sd	a4,168(sp)

0000000000002200 <.L57>:
    2200:	02013783          	ld	a5,32(sp)
    2204:	40e787bb          	subw	a5,a5,a4
    2208:	02f05463          	blez	a5,2230 <.L58>
    220c:	00ca2703          	lw	a4,12(s4)
    2210:	00877713          	andi	a4,a4,8
    2214:	6c070e63          	beqz	a4,28f0 <.L59>
    2218:	00f43423          	sd	a5,8(s0)
    221c:	0a000713          	li	a4,160
    2220:	d4318793          	addi	a5,gp,-701 # 5c23 <fmt+0x3>
    2224:	00e43023          	sd	a4,0(s0)
    2228:	00f43823          	sd	a5,16(s0)
    222c:	01840413          	addi	s0,s0,24

0000000000002230 <.L58>:
    2230:	0a010913          	addi	s2,sp,160
    2234:	00000493          	li	s1,0

0000000000002238 <.L62>:
    2238:	00893983          	ld	s3,8(s2)
    223c:	01093683          	ld	a3,16(s2)
    2240:	00093583          	ld	a1,0(s2)
    2244:	04013503          	ld	a0,64(sp)
    2248:	04813783          	ld	a5,72(sp)
    224c:	00098613          	mv	a2,s3
    2250:	000780e7          	jalr	a5
    2254:	6d351463          	bne	a0,s3,291c <.L81>
    2258:	01890913          	addi	s2,s2,24
    225c:	009504bb          	addw	s1,a0,s1
    2260:	fc896ce3          	bltu	s2,s0,2238 <.L62>
    2264:	00048513          	mv	a0,s1

0000000000002268 <.L1>:
    2268:	1c813083          	ld	ra,456(sp)
    226c:	1c013403          	ld	s0,448(sp)
    2270:	1b813483          	ld	s1,440(sp)
    2274:	1b013903          	ld	s2,432(sp)
    2278:	1a813983          	ld	s3,424(sp)
    227c:	1a013a03          	ld	s4,416(sp)
    2280:	19813a83          	ld	s5,408(sp)
    2284:	19013b03          	ld	s6,400(sp)
    2288:	18813b83          	ld	s7,392(sp)
    228c:	18013c03          	ld	s8,384(sp)
    2290:	17813c83          	ld	s9,376(sp)
    2294:	17013d03          	ld	s10,368(sp)
    2298:	16813d83          	ld	s11,360(sp)
    229c:	1d010113          	addi	sp,sp,464
    22a0:	00008067          	ret

00000000000022a4 <.L2>:
    22a4:	00013c23          	sd	zero,24(sp)
    22a8:	e409dce3          	bgez	s3,2100 <.L3>
    22ac:	00600993          	li	s3,6
    22b0:	e51ff06f          	j	2100 <.L3>

00000000000022b4 <.L4>:
    22b4:	0017f793          	andi	a5,a5,1
    22b8:	e60780e3          	beqz	a5,2118 <.L5>
    22bc:	02000793          	li	a5,32
    22c0:	e55ff06f          	j	2114 <.L116>

00000000000022c4 <.L110>:
    22c4:	00000613          	li	a2,0
    22c8:	00000693          	li	a3,0
    22cc:	00040513          	mv	a0,s0
    22d0:	000b8593          	mv	a1,s7
    22d4:	6a4010ef          	jal	ra,3978 <__eqtf2>
    22d8:	04051c63          	bnez	a0,2330 <.L111>
    22dc:	00040513          	mv	a0,s0
    22e0:	000b8593          	mv	a1,s7
    22e4:	7ad020ef          	jal	ra,5290 <__trunctfdf2>
    22e8:	e20507d3          	fmv.x.d	a5,fa0
    22ec:	00004717          	auipc	a4,0x4
    22f0:	67470713          	addi	a4,a4,1652 # 6960 <.LC2>
    22f4:	00078693          	mv	a3,a5
    22f8:	0006869b          	sext.w	a3,a3
    22fc:	4207d793          	srai	a5,a5,0x20
    2300:	00d78663          	beq	a5,a3,230c <.L65>
    2304:	02d00793          	li	a5,45
    2308:	06f10023          	sb	a5,96(sp)

000000000000230c <.L65>:
    230c:	00900793          	li	a5,9
    2310:	00f13423          	sd	a5,8(sp)
    2314:	00073783          	ld	a5,0(a4)
    2318:	00873c03          	ld	s8,8(a4)
    231c:	fff00c93          	li	s9,-1
    2320:	02400493          	li	s1,36
    2324:	00a00a93          	li	s5,10
    2328:	00f13023          	sd	a5,0(sp)
    232c:	17c0006f          	j	24a8 <.L11>

0000000000002330 <.L111>:
    2330:	00000613          	li	a2,0
    2334:	00000693          	li	a3,0
    2338:	00040513          	mv	a0,s0
    233c:	000b8593          	mv	a1,s7
    2340:	7f8010ef          	jal	ra,3b38 <__letf2>
    2344:	2c055863          	bgez	a0,2614 <.L112>
    2348:	fff00b13          	li	s6,-1
    234c:	02d00793          	li	a5,45
    2350:	03fb1b13          	slli	s6,s6,0x3f
    2354:	06f10023          	sb	a5,96(sp)
    2358:	02813c23          	sd	s0,56(sp)
    235c:	016bcb33          	xor	s6,s7,s6

0000000000002360 <.L12>:
    2360:	00004797          	auipc	a5,0x4
    2364:	62078793          	addi	a5,a5,1568 # 6980 <.LC4>
    2368:	0007b603          	ld	a2,0(a5)
    236c:	0087b683          	ld	a3,8(a5)
    2370:	00040513          	mv	a0,s0
    2374:	000b0593          	mv	a1,s6
    2378:	0a5010ef          	jal	ra,3c1c <__multf3>
    237c:	00040613          	mv	a2,s0
    2380:	000b0693          	mv	a3,s6
    2384:	5f4010ef          	jal	ra,3978 <__eqtf2>
    2388:	da050ce3          	beqz	a0,2140 <.L8>
    238c:	06100793          	li	a5,97
    2390:	28f48863          	beq	s1,a5,2620 <.L67>
    2394:	00004717          	auipc	a4,0x4
    2398:	5cc70713          	addi	a4,a4,1484 # 6960 <.LC2>
    239c:	00900793          	li	a5,9
    23a0:	00f13423          	sd	a5,8(sp)
    23a4:	00073783          	ld	a5,0(a4)
    23a8:	00873c03          	ld	s8,8(a4)
    23ac:	00800c93          	li	s9,8
    23b0:	02400493          	li	s1,36
    23b4:	c7018d13          	addi	s10,gp,-912 # 5b50 <exp10_table>
    23b8:	00a00a93          	li	s5,10
    23bc:	00f13023          	sd	a5,0(sp)
    23c0:	00004717          	auipc	a4,0x4
    23c4:	5b070713          	addi	a4,a4,1456 # 6970 <.LC3>

00000000000023c8 <.L117>:
    23c8:	00073783          	ld	a5,0(a4)
    23cc:	0c0d0d93          	addi	s11,s10,192
    23d0:	000b0b93          	mv	s7,s6
    23d4:	02f13423          	sd	a5,40(sp)
    23d8:	00873783          	ld	a5,8(a4)
    23dc:	02f13823          	sd	a5,48(sp)
    23e0:	000017b7          	lui	a5,0x1
    23e4:	00f13823          	sd	a5,16(sp)

00000000000023e8 <.L24>:
    23e8:	000db303          	ld	t1,0(s11)
    23ec:	008db883          	ld	a7,8(s11)
    23f0:	02813603          	ld	a2,40(sp)
    23f4:	03013683          	ld	a3,48(sp)
    23f8:	03813503          	ld	a0,56(sp)
    23fc:	000b0593          	mv	a1,s6
    2400:	04613c23          	sd	t1,88(sp)
    2404:	05113823          	sd	a7,80(sp)
    2408:	730010ef          	jal	ra,3b38 <__letf2>
    240c:	05013883          	ld	a7,80(sp)
    2410:	05813303          	ld	t1,88(sp)
    2414:	00088693          	mv	a3,a7
    2418:	00030613          	mv	a2,t1
    241c:	24055263          	bgez	a0,2660 <.L114>
    2420:	00040513          	mv	a0,s0
    2424:	000b8593          	mv	a1,s7
    2428:	7f4010ef          	jal	ra,3c1c <__multf3>
    242c:	00013603          	ld	a2,0(sp)
    2430:	000c0693          	mv	a3,s8
    2434:	04a13c23          	sd	a0,88(sp)
    2438:	04b13823          	sd	a1,80(sp)
    243c:	6fc010ef          	jal	ra,3b38 <__letf2>
    2440:	00055e63          	bgez	a0,245c <.L21>
    2444:	01013783          	ld	a5,16(sp)
    2448:	40fc8cbb          	subw	s9,s9,a5

000000000000244c <.L118>:
    244c:	05813303          	ld	t1,88(sp)
    2450:	05013883          	ld	a7,80(sp)
    2454:	00030413          	mv	s0,t1
    2458:	00088b93          	mv	s7,a7

000000000000245c <.L21>:
    245c:	01013783          	ld	a5,16(sp)
    2460:	ff0d8693          	addi	a3,s11,-16
    2464:	4017d793          	srai	a5,a5,0x1
    2468:	00f13823          	sd	a5,16(sp)
    246c:	1fbd1663          	bne	s10,s11,2658 <.L68>
    2470:	00013603          	ld	a2,0(sp)
    2474:	000c0693          	mv	a3,s8
    2478:	00040513          	mv	a0,s0
    247c:	000b8593          	mv	a1,s7
    2480:	5d4010ef          	jal	ra,3a54 <__getf2>
    2484:	02054263          	bltz	a0,24a8 <.L11>
    2488:	000d3603          	ld	a2,0(s10)
    248c:	008d3683          	ld	a3,8(s10)
    2490:	00040513          	mv	a0,s0
    2494:	000b8593          	mv	a1,s7
    2498:	3b5000ef          	jal	ra,304c <__divtf3>
    249c:	00050413          	mv	s0,a0
    24a0:	00058b93          	mv	s7,a1
    24a4:	001c8c9b          	addiw	s9,s9,1

00000000000024a8 <.L11>:
    24a8:	07a10d13          	addi	s10,sp,122
    24ac:	00400893          	li	a7,4
    24b0:	000d0b13          	mv	s6,s10

00000000000024b4 <.L27>:
    24b4:	000b8593          	mv	a1,s7
    24b8:	00040513          	mv	a0,s0
    24bc:	01113823          	sd	a7,16(sp)
    24c0:	33d020ef          	jal	ra,4ffc <__fixunstfdi>
    24c4:	00050d93          	mv	s11,a0
    24c8:	41d020ef          	jal	ra,50e4 <__floatunditf>
    24cc:	00050613          	mv	a2,a0
    24d0:	00058693          	mv	a3,a1
    24d4:	00040513          	mv	a0,s0
    24d8:	000b8593          	mv	a1,s7
    24dc:	7e1010ef          	jal	ra,44bc <__subtf3>
    24e0:	00013603          	ld	a2,0(sp)
    24e4:	000c0693          	mv	a3,s8
    24e8:	734010ef          	jal	ra,3c1c <__multf3>
    24ec:	00813783          	ld	a5,8(sp)
    24f0:	01013883          	ld	a7,16(sp)
    24f4:	00050413          	mv	s0,a0
    24f8:	00fd0d33          	add	s10,s10,a5
    24fc:	00058b93          	mv	s7,a1
    2500:	000d0713          	mv	a4,s10

0000000000002504 <.L26>:
    2504:	035df6b3          	remu	a3,s11,s5
    2508:	00813783          	ld	a5,8(sp)
    250c:	fff70713          	addi	a4,a4,-1
    2510:	0306869b          	addiw	a3,a3,48
    2514:	00d70023          	sb	a3,0(a4)
    2518:	40ed06bb          	subw	a3,s10,a4
    251c:	035dddb3          	divu	s11,s11,s5
    2520:	fef6c2e3          	blt	a3,a5,2504 <.L26>
    2524:	fff8889b          	addiw	a7,a7,-1
    2528:	f80896e3          	bnez	a7,24b4 <.L27>
    252c:	06000713          	li	a4,96
    2530:	01276c63          	bltu	a4,s2,2548 <.L28>
    2534:	06814703          	lbu	a4,104(sp)
    2538:	0209091b          	addiw	s2,s2,32
    253c:	0ff97913          	zext.b	s2,s2
    2540:	fe07071b          	addiw	a4,a4,-32
    2544:	06e10423          	sb	a4,104(sp)

0000000000002548 <.L28>:
    2548:	06700713          	li	a4,103
    254c:	14e91263          	bne	s2,a4,2690 <.L29>
    2550:	00000613          	li	a2,0
    2554:	00098663          	beqz	s3,2560 <.L30>
    2558:	fff9899b          	addiw	s3,s3,-1
    255c:	00098613          	mv	a2,s3

0000000000002560 <.L30>:
    2560:	00003737          	lui	a4,0x3
    2564:	06e11c23          	sh	a4,120(sp)
    2568:	07910593          	addi	a1,sp,121
    256c:	14964c63          	blt	a2,s1,26c4 <.L31>
    2570:	00148493          	addi	s1,s1,1
    2574:	009584b3          	add	s1,a1,s1
    2578:	00000713          	li	a4,0

000000000000257c <.L32>:
    257c:	03000513          	li	a0,48
    2580:	02fa8a93          	addi	s5,s5,47

0000000000002584 <.L103>:
    2584:	fff4c683          	lbu	a3,-1(s1)
    2588:	fff48493          	addi	s1,s1,-1
    258c:	00e686bb          	addw	a3,a3,a4
    2590:	0ff6f693          	zext.b	a3,a3
    2594:	00d48023          	sb	a3,0(s1)
    2598:	fea686e3          	beq	a3,a0,2584 <.L103>
    259c:	fedae4e3          	bltu	s5,a3,2584 <.L103>
    25a0:	02096713          	ori	a4,s2,32
    25a4:	06100693          	li	a3,97
    25a8:	14d71e63          	bne	a4,a3,2704 <.L34>
    25ac:	06814683          	lbu	a3,104(sp)
    25b0:	00048513          	mv	a0,s1
    25b4:	03900813          	li	a6,57
    25b8:	fb76869b          	addiw	a3,a3,-73
    25bc:	0ff6f693          	zext.b	a3,a3

00000000000025c0 <.L35>:
    25c0:	00054703          	lbu	a4,0(a0)
    25c4:	12071263          	bnez	a4,26e8 <.L37>
    25c8:	1295ea63          	bltu	a1,s1,26fc <.L38>

00000000000025cc <.L40>:
    25cc:	001c8c9b          	addiw	s9,s9,1
    25d0:	00058493          	mv	s1,a1
    25d4:	00058b13          	mv	s6,a1

00000000000025d8 <.L39>:
    25d8:	000480a3          	sb	zero,1(s1)
    25dc:	06700713          	li	a4,103
    25e0:	12e91663          	bne	s2,a4,270c <.L41>
    25e4:	ffc00713          	li	a4,-4
    25e8:	2eecc063          	blt	s9,a4,28c8 <.L72>
    25ec:	2d964e63          	blt	a2,s9,28c8 <.L72>
    25f0:	419609bb          	subw	s3,a2,s9

00000000000025f4 <.L43>:
    25f4:	000c8713          	mv	a4,s9
    25f8:	000cda63          	bgez	s9,260c <.L119>
    25fc:	03000713          	li	a4,48
    2600:	feeb0fa3          	sb	a4,-1(s6)
    2604:	000c8713          	mv	a4,s9
    2608:	fffb0b13          	addi	s6,s6,-1

000000000000260c <.L119>:
    260c:	06600693          	li	a3,102
    2610:	10c0006f          	j	271c <.L42>

0000000000002614 <.L112>:
    2614:	02813c23          	sd	s0,56(sp)
    2618:	000b8b13          	mv	s6,s7
    261c:	d45ff06f          	j	2360 <.L12>

0000000000002620 <.L67>:
    2620:	00004717          	auipc	a4,0x4
    2624:	32070713          	addi	a4,a4,800 # 6940 <.LC0>
    2628:	00800793          	li	a5,8
    262c:	00f13423          	sd	a5,8(sp)
    2630:	00073783          	ld	a5,0(a4)
    2634:	00873c03          	ld	s8,8(a4)
    2638:	00700c93          	li	s9,7
    263c:	01d00493          	li	s1,29
    2640:	ba018d13          	addi	s10,gp,-1120 # 5a80 <exp16_table>
    2644:	01000a93          	li	s5,16
    2648:	00f13023          	sd	a5,0(sp)
    264c:	00004717          	auipc	a4,0x4
    2650:	30470713          	addi	a4,a4,772 # 6950 <.LC1>
    2654:	d75ff06f          	j	23c8 <.L117>

0000000000002658 <.L68>:
    2658:	00068d93          	mv	s11,a3
    265c:	d8dff06f          	j	23e8 <.L24>

0000000000002660 <.L114>:
    2660:	00040513          	mv	a0,s0
    2664:	000b8593          	mv	a1,s7
    2668:	1e5000ef          	jal	ra,304c <__divtf3>
    266c:	02813603          	ld	a2,40(sp)
    2670:	03013683          	ld	a3,48(sp)
    2674:	04a13c23          	sd	a0,88(sp)
    2678:	04b13823          	sd	a1,80(sp)
    267c:	3d8010ef          	jal	ra,3a54 <__getf2>
    2680:	dc054ee3          	bltz	a0,245c <.L21>
    2684:	01013783          	ld	a5,16(sp)
    2688:	00fc8cbb          	addw	s9,s9,a5
    268c:	dc1ff06f          	j	244c <.L118>

0000000000002690 <.L29>:
    2690:	06600713          	li	a4,102
    2694:	00098613          	mv	a2,s3
    2698:	ece914e3          	bne	s2,a4,2560 <.L30>
    269c:	013c863b          	addw	a2,s9,s3
    26a0:	fff00713          	li	a4,-1
    26a4:	eae65ee3          	bge	a2,a4,2560 <.L30>
    26a8:	02400613          	li	a2,36
    26ac:	03000593          	li	a1,48
    26b0:	07810513          	addi	a0,sp,120
    26b4:	904ff0ef          	jal	ra,17b8 <__GI_memset>
    26b8:	fff00c93          	li	s9,-1
    26bc:	fff00613          	li	a2,-1
    26c0:	ea1ff06f          	j	2560 <.L30>

00000000000026c4 <.L31>:
    26c4:	0026049b          	addiw	s1,a2,2
    26c8:	009584b3          	add	s1,a1,s1
    26cc:	0004c703          	lbu	a4,0(s1)
    26d0:	001ad693          	srli	a3,s5,0x1
    26d4:	03068693          	addi	a3,a3,48
    26d8:	00d73733          	sltu	a4,a4,a3
    26dc:	00174713          	xori	a4,a4,1
    26e0:	0007071b          	sext.w	a4,a4
    26e4:	e99ff06f          	j	257c <.L32>

00000000000026e8 <.L37>:
    26e8:	00e87663          	bgeu	a6,a4,26f4 <.L36>
    26ec:	00d7073b          	addw	a4,a4,a3
    26f0:	00e50023          	sb	a4,0(a0)

00000000000026f4 <.L36>:
    26f4:	fff50513          	addi	a0,a0,-1
    26f8:	ec9ff06f          	j	25c0 <.L35>

00000000000026fc <.L38>:
    26fc:	002c9c9b          	slliw	s9,s9,0x2
    2700:	ed9ff06f          	j	25d8 <.L39>

0000000000002704 <.L34>:
    2704:	ec95f4e3          	bgeu	a1,s1,25cc <.L40>
    2708:	ed1ff06f          	j	25d8 <.L39>

000000000000270c <.L41>:
    270c:	06600713          	li	a4,102
    2710:	00090693          	mv	a3,s2
    2714:	eee900e3          	beq	s2,a4,25f4 <.L43>

0000000000002718 <.L120>:
    2718:	00000713          	li	a4,0

000000000000271c <.L42>:
    271c:	000b4503          	lbu	a0,0(s6)
    2720:	00100593          	li	a1,1
    2724:	0b000613          	li	a2,176
    2728:	0cb13023          	sd	a1,192(sp)
    272c:	06410593          	addi	a1,sp,100
    2730:	0cb13423          	sd	a1,200(sp)
    2734:	0ac13c23          	sd	a2,184(sp)
    2738:	06a10223          	sb	a0,100(sp)
    273c:	060102a3          	sb	zero,101(sp)
    2740:	001b0593          	addi	a1,s6,1
    2744:	0d010413          	addi	s0,sp,208
    2748:	41648b3b          	subw	s6,s1,s6
    274c:	02074e63          	bltz	a4,2788 <.L44>
    2750:	07000513          	li	a0,112
    2754:	0ca13823          	sd	a0,208(sp)
    2758:	0eb13023          	sd	a1,224(sp)
    275c:	17674a63          	blt	a4,s6,28d0 <.L45>
    2760:	0d613c23          	sd	s6,216(sp)
    2764:	41670b3b          	subw	s6,a4,s6
    2768:	160b0c63          	beqz	s6,28e0 <.L76>
    276c:	d4318713          	addi	a4,gp,-701 # 5c23 <fmt+0x3>
    2770:	0f613823          	sd	s6,240(sp)
    2774:	0ec13423          	sd	a2,232(sp)
    2778:	0ee13c23          	sd	a4,248(sp)
    277c:	00000b13          	li	s6,0
    2780:	10010413          	addi	s0,sp,256

0000000000002784 <.L77>:
    2784:	fff00713          	li	a4,-1

0000000000002788 <.L44>:
    2788:	00ca2603          	lw	a2,12(s4)
    278c:	01067613          	andi	a2,a2,16
    2790:	00cb6533          	or	a0,s6,a2
    2794:	00051c63          	bnez	a0,27ac <.L46>
    2798:	06700513          	li	a0,103
    279c:	02a90663          	beq	s2,a0,27c8 <.L47>
    27a0:	06100513          	li	a0,97
    27a4:	02a90263          	beq	s2,a0,27c8 <.L47>
    27a8:	03305063          	blez	s3,27c8 <.L47>

00000000000027ac <.L46>:
    27ac:	07000513          	li	a0,112
    27b0:	00a43023          	sd	a0,0(s0)
    27b4:	00100513          	li	a0,1
    27b8:	00a43423          	sd	a0,8(s0)
    27bc:	d5018513          	addi	a0,gp,-688 # 5c30 <fmt+0x10>
    27c0:	00a43823          	sd	a0,16(s0)
    27c4:	01840413          	addi	s0,s0,24

00000000000027c8 <.L47>:
    27c8:	0017081b          	addiw	a6,a4,1
    27cc:	00080513          	mv	a0,a6
    27d0:	02080263          	beqz	a6,27f4 <.L48>
    27d4:	fff74713          	not	a4,a4
    27d8:	0007071b          	sext.w	a4,a4
    27dc:	00e43423          	sd	a4,8(s0)
    27e0:	0b000813          	li	a6,176
    27e4:	d4318713          	addi	a4,gp,-701 # 5c23 <fmt+0x3>
    27e8:	01043023          	sd	a6,0(s0)
    27ec:	00e43823          	sd	a4,16(s0)
    27f0:	01840413          	addi	s0,s0,24

00000000000027f4 <.L48>:
    27f4:	000b0c63          	beqz	s6,280c <.L49>
    27f8:	07000713          	li	a4,112
    27fc:	00e43023          	sd	a4,0(s0)
    2800:	01643423          	sd	s6,8(s0)
    2804:	00b43823          	sd	a1,16(s0)
    2808:	01840413          	addi	s0,s0,24

000000000000280c <.L49>:
    280c:	06700713          	li	a4,103
    2810:	00e91463          	bne	s2,a4,2818 <.L50>
    2814:	02060a63          	beqz	a2,2848 <.L51>

0000000000002818 <.L50>:
    2818:	01813783          	ld	a5,24(sp)
    281c:	02079663          	bnez	a5,2848 <.L51>
    2820:	40ab073b          	subw	a4,s6,a0
    2824:	00070b13          	mv	s6,a4
    2828:	03375063          	bge	a4,s3,2848 <.L51>
    282c:	0b000713          	li	a4,176
    2830:	00e43023          	sd	a4,0(s0)
    2834:	416989bb          	subw	s3,s3,s6
    2838:	d4318713          	addi	a4,gp,-701 # 5c23 <fmt+0x3>
    283c:	01343423          	sd	s3,8(s0)
    2840:	00e43823          	sd	a4,16(s0)
    2844:	01840413          	addi	s0,s0,24

0000000000002848 <.L51>:
    2848:	06600713          	li	a4,102
    284c:	92e68ce3          	beq	a3,a4,2184 <.L17>
    2850:	06100713          	li	a4,97
    2854:	06814303          	lbu	t1,104(sp)
    2858:	00400813          	li	a6,4
    285c:	00e91463          	bne	s2,a4,2864 <.L52>
    2860:	00300813          	li	a6,3

0000000000002864 <.L52>:
    2864:	02b00513          	li	a0,43
    2868:	000cd663          	bgez	s9,2874 <.L53>
    286c:	41900cbb          	negw	s9,s9
    2870:	02d00513          	li	a0,45

0000000000002874 <.L53>:
    2874:	06010ba3          	sb	zero,119(sp)
    2878:	00200613          	li	a2,2
    287c:	07710713          	addi	a4,sp,119
    2880:	00a00893          	li	a7,10

0000000000002884 <.L54>:
    2884:	031ce5bb          	remw	a1,s9,a7
    2888:	fff70713          	addi	a4,a4,-1
    288c:	0016061b          	addiw	a2,a2,1
    2890:	0305859b          	addiw	a1,a1,48
    2894:	00b70023          	sb	a1,0(a4)
    2898:	031cccbb          	divw	s9,s9,a7
    289c:	05064663          	blt	a2,a6,28e8 <.L80>
    28a0:	fe0c92e3          	bnez	s9,2884 <.L54>
    28a4:	fea70fa3          	sb	a0,-1(a4)
    28a8:	fe670f23          	sb	t1,-2(a4)
    28ac:	07000793          	li	a5,112
    28b0:	ffe70713          	addi	a4,a4,-2
    28b4:	00f43023          	sd	a5,0(s0)
    28b8:	00c43423          	sd	a2,8(s0)
    28bc:	00e43823          	sd	a4,16(s0)
    28c0:	01840413          	addi	s0,s0,24
    28c4:	8c1ff06f          	j	2184 <.L17>

00000000000028c8 <.L72>:
    28c8:	06700693          	li	a3,103
    28cc:	e4dff06f          	j	2718 <.L120>

00000000000028d0 <.L45>:
    28d0:	ea070ae3          	beqz	a4,2784 <.L77>
    28d4:	0ce13c23          	sd	a4,216(sp)
    28d8:	00e585b3          	add	a1,a1,a4
    28dc:	40eb0b3b          	subw	s6,s6,a4

00000000000028e0 <.L76>:
    28e0:	0e810413          	addi	s0,sp,232
    28e4:	ea1ff06f          	j	2784 <.L77>

00000000000028e8 <.L80>:
    28e8:	00300613          	li	a2,3
    28ec:	f99ff06f          	j	2884 <.L54>

00000000000028f0 <.L59>:
    28f0:	010a2603          	lw	a2,16(s4)
    28f4:	03000693          	li	a3,48
    28f8:	0a813703          	ld	a4,168(sp)
    28fc:	00d61a63          	bne	a2,a3,2910 <.L60>
    2900:	0c013703          	ld	a4,192(sp)
    2904:	00f707b3          	add	a5,a4,a5
    2908:	0cf13023          	sd	a5,192(sp)
    290c:	925ff06f          	j	2230 <.L58>

0000000000002910 <.L60>:
    2910:	00e787b3          	add	a5,a5,a4
    2914:	0af13423          	sd	a5,168(sp)
    2918:	919ff06f          	j	2230 <.L58>

000000000000291c <.L81>:
    291c:	fff00513          	li	a0,-1
    2920:	949ff06f          	j	2268 <.L1>

0000000000002924 <__GI_memchr>:
    2924:	0ff5f713          	zext.b	a4,a1

0000000000002928 <.L2>:
    2928:	00060663          	beqz	a2,2934 <.L4>
    292c:	00757793          	andi	a5,a0,7
    2930:	04079263          	bnez	a5,2974 <.L5>

0000000000002934 <.L4>:
    2934:	0ff5f593          	zext.b	a1,a1
    2938:	00859693          	slli	a3,a1,0x8
    293c:	00b6e6b3          	or	a3,a3,a1
    2940:	01069793          	slli	a5,a3,0x10
    2944:	00d7e7b3          	or	a5,a5,a3
    2948:	02079813          	slli	a6,a5,0x20
    294c:	00f86833          	or	a6,a6,a5
    2950:	00700313          	li	t1,7
    2954:	00004897          	auipc	a7,0x4
    2958:	03c8b883          	ld	a7,60(a7) # 6990 <.LC0>
    295c:	fff00e13          	li	t3,-1

0000000000002960 <.L6>:
    2960:	02c36463          	bltu	t1,a2,2988 <.L14>
    2964:	00c50633          	add	a2,a0,a2

0000000000002968 <.L15>:
    2968:	0cc51063          	bne	a0,a2,2a28 <.L16>
    296c:	00000513          	li	a0,0

0000000000002970 <.L1>:
    2970:	00008067          	ret

0000000000002974 <.L5>:
    2974:	00054783          	lbu	a5,0(a0)
    2978:	fee78ce3          	beq	a5,a4,2970 <.L1>
    297c:	fff60613          	addi	a2,a2,-1
    2980:	00150513          	addi	a0,a0,1
    2984:	fa5ff06f          	j	2928 <.L2>

0000000000002988 <.L14>:
    2988:	00053583          	ld	a1,0(a0)
    298c:	00850793          	addi	a5,a0,8
    2990:	00b845b3          	xor	a1,a6,a1
    2994:	011586b3          	add	a3,a1,a7
    2998:	00b6c6b3          	xor	a3,a3,a1
    299c:	0116e6b3          	or	a3,a3,a7
    29a0:	07c68e63          	beq	a3,t3,2a1c <.L7>
    29a4:	ff87c683          	lbu	a3,-8(a5) # ff8 <.L39+0x4>
    29a8:	fce684e3          	beq	a3,a4,2970 <.L1>
    29ac:	ff97c683          	lbu	a3,-7(a5)
    29b0:	00e69663          	bne	a3,a4,29bc <.L8>
    29b4:	00150513          	addi	a0,a0,1
    29b8:	00008067          	ret

00000000000029bc <.L8>:
    29bc:	ffa7c683          	lbu	a3,-6(a5)
    29c0:	00e69663          	bne	a3,a4,29cc <.L9>
    29c4:	00250513          	addi	a0,a0,2
    29c8:	00008067          	ret

00000000000029cc <.L9>:
    29cc:	ffb7c683          	lbu	a3,-5(a5)
    29d0:	00e69663          	bne	a3,a4,29dc <.L10>
    29d4:	00350513          	addi	a0,a0,3
    29d8:	00008067          	ret

00000000000029dc <.L10>:
    29dc:	ffc7c683          	lbu	a3,-4(a5)
    29e0:	00e69663          	bne	a3,a4,29ec <.L11>
    29e4:	00450513          	addi	a0,a0,4
    29e8:	00008067          	ret

00000000000029ec <.L11>:
    29ec:	ffd7c683          	lbu	a3,-3(a5)
    29f0:	00e69663          	bne	a3,a4,29fc <.L12>
    29f4:	00550513          	addi	a0,a0,5
    29f8:	00008067          	ret

00000000000029fc <.L12>:
    29fc:	ffe7c683          	lbu	a3,-2(a5)
    2a00:	00e69663          	bne	a3,a4,2a0c <.L13>
    2a04:	00650513          	addi	a0,a0,6
    2a08:	00008067          	ret

0000000000002a0c <.L13>:
    2a0c:	fff7c683          	lbu	a3,-1(a5)
    2a10:	00e69663          	bne	a3,a4,2a1c <.L7>
    2a14:	fff78513          	addi	a0,a5,-1
    2a18:	00008067          	ret

0000000000002a1c <.L7>:
    2a1c:	ff860613          	addi	a2,a2,-8
    2a20:	00078513          	mv	a0,a5
    2a24:	f3dff06f          	j	2960 <.L6>

0000000000002a28 <.L16>:
    2a28:	00054783          	lbu	a5,0(a0)
    2a2c:	f4e782e3          	beq	a5,a4,2970 <.L1>
    2a30:	00150513          	addi	a0,a0,1
    2a34:	f35ff06f          	j	2968 <.L15>

0000000000002a38 <__GI_memrchr>:
    2a38:	0ff5f793          	zext.b	a5,a1
    2a3c:	00c50533          	add	a0,a0,a2

0000000000002a40 <.L2>:
    2a40:	00060663          	beqz	a2,2a4c <.L4>
    2a44:	00757713          	andi	a4,a0,7
    2a48:	04071463          	bnez	a4,2a90 <.L5>

0000000000002a4c <.L4>:
    2a4c:	0ff5f593          	zext.b	a1,a1
    2a50:	00859693          	slli	a3,a1,0x8
    2a54:	00b6e6b3          	or	a3,a3,a1
    2a58:	01069713          	slli	a4,a3,0x10
    2a5c:	00d76733          	or	a4,a4,a3
    2a60:	02071813          	slli	a6,a4,0x20
    2a64:	00e86833          	or	a6,a6,a4
    2a68:	00700313          	li	t1,7
    2a6c:	00050713          	mv	a4,a0
    2a70:	00004897          	auipc	a7,0x4
    2a74:	f208b883          	ld	a7,-224(a7) # 6990 <.LC0>
    2a78:	fff00e13          	li	t3,-1

0000000000002a7c <.L6>:
    2a7c:	02c36463          	bltu	t1,a2,2aa4 <.L15>
    2a80:	40c50633          	sub	a2,a0,a2

0000000000002a84 <.L16>:
    2a84:	0cc51063          	bne	a0,a2,2b44 <.L17>
    2a88:	00000513          	li	a0,0
    2a8c:	0c40006f          	j	2b50 <.L1>

0000000000002a90 <.L5>:
    2a90:	fff54703          	lbu	a4,-1(a0)
    2a94:	fff50513          	addi	a0,a0,-1
    2a98:	0af70c63          	beq	a4,a5,2b50 <.L1>
    2a9c:	fff60613          	addi	a2,a2,-1
    2aa0:	fa1ff06f          	j	2a40 <.L2>

0000000000002aa4 <.L15>:
    2aa4:	ff853583          	ld	a1,-8(a0)
    2aa8:	ff850513          	addi	a0,a0,-8
    2aac:	00b845b3          	xor	a1,a6,a1
    2ab0:	011586b3          	add	a3,a1,a7
    2ab4:	00b6c6b3          	xor	a3,a3,a1
    2ab8:	0116e6b3          	or	a3,a3,a7
    2abc:	07c68e63          	beq	a3,t3,2b38 <.L7>
    2ac0:	00754683          	lbu	a3,7(a0)
    2ac4:	00f69663          	bne	a3,a5,2ad0 <.L8>
    2ac8:	fff70513          	addi	a0,a4,-1
    2acc:	00008067          	ret

0000000000002ad0 <.L8>:
    2ad0:	00654683          	lbu	a3,6(a0)
    2ad4:	00f69663          	bne	a3,a5,2ae0 <.L9>
    2ad8:	ffe70513          	addi	a0,a4,-2
    2adc:	00008067          	ret

0000000000002ae0 <.L9>:
    2ae0:	00554683          	lbu	a3,5(a0)
    2ae4:	00f69663          	bne	a3,a5,2af0 <.L10>
    2ae8:	ffd70513          	addi	a0,a4,-3
    2aec:	00008067          	ret

0000000000002af0 <.L10>:
    2af0:	00454683          	lbu	a3,4(a0)
    2af4:	00f69663          	bne	a3,a5,2b00 <.L11>
    2af8:	ffc70513          	addi	a0,a4,-4
    2afc:	00008067          	ret

0000000000002b00 <.L11>:
    2b00:	00354683          	lbu	a3,3(a0)
    2b04:	00f69663          	bne	a3,a5,2b10 <.L12>
    2b08:	ffb70513          	addi	a0,a4,-5
    2b0c:	00008067          	ret

0000000000002b10 <.L12>:
    2b10:	00254683          	lbu	a3,2(a0)
    2b14:	00f69663          	bne	a3,a5,2b20 <.L13>
    2b18:	ffa70513          	addi	a0,a4,-6
    2b1c:	00008067          	ret

0000000000002b20 <.L13>:
    2b20:	00154683          	lbu	a3,1(a0)
    2b24:	00f69663          	bne	a3,a5,2b30 <.L14>
    2b28:	ff970513          	addi	a0,a4,-7
    2b2c:	00008067          	ret

0000000000002b30 <.L14>:
    2b30:	00054703          	lbu	a4,0(a0)
    2b34:	00f70e63          	beq	a4,a5,2b50 <.L1>

0000000000002b38 <.L7>:
    2b38:	ff860613          	addi	a2,a2,-8
    2b3c:	00050713          	mv	a4,a0
    2b40:	f3dff06f          	j	2a7c <.L6>

0000000000002b44 <.L17>:
    2b44:	fff54703          	lbu	a4,-1(a0)
    2b48:	fff50513          	addi	a0,a0,-1
    2b4c:	f2f71ce3          	bne	a4,a5,2a84 <.L16>

0000000000002b50 <.L1>:
    2b50:	00008067          	ret

0000000000002b54 <__GI_strrchr>:
    2b54:	fe010113          	addi	sp,sp,-32
    2b58:	00813823          	sd	s0,16(sp)
    2b5c:	00113c23          	sd	ra,24(sp)
    2b60:	00913423          	sd	s1,8(sp)
    2b64:	0ff5f413          	zext.b	s0,a1
    2b68:	04041663          	bnez	s0,2bb4 <.L6>
    2b6c:	01013403          	ld	s0,16(sp)
    2b70:	01813083          	ld	ra,24(sp)
    2b74:	00813483          	ld	s1,8(sp)
    2b78:	00000593          	li	a1,0
    2b7c:	02010113          	addi	sp,sp,32
    2b80:	0740006f          	j	2bf4 <__GI_strchr>

0000000000002b84 <.L3>:
    2b84:	00150793          	addi	a5,a0,1
    2b88:	00050493          	mv	s1,a0

0000000000002b8c <.L2>:
    2b8c:	00040593          	mv	a1,s0
    2b90:	00078513          	mv	a0,a5
    2b94:	060000ef          	jal	ra,2bf4 <__GI_strchr>
    2b98:	fe0516e3          	bnez	a0,2b84 <.L3>
    2b9c:	01813083          	ld	ra,24(sp)
    2ba0:	01013403          	ld	s0,16(sp)
    2ba4:	00048513          	mv	a0,s1
    2ba8:	00813483          	ld	s1,8(sp)
    2bac:	02010113          	addi	sp,sp,32
    2bb0:	00008067          	ret

0000000000002bb4 <.L6>:
    2bb4:	00050793          	mv	a5,a0
    2bb8:	00000493          	li	s1,0
    2bbc:	fd1ff06f          	j	2b8c <.L2>

0000000000002bc0 <__GI_mempcpy>:
    2bc0:	fe010113          	addi	sp,sp,-32
    2bc4:	00813823          	sd	s0,16(sp)
    2bc8:	00913423          	sd	s1,8(sp)
    2bcc:	00113c23          	sd	ra,24(sp)
    2bd0:	00050493          	mv	s1,a0
    2bd4:	00060413          	mv	s0,a2
    2bd8:	845fe0ef          	jal	ra,141c <__GI_memcpy>
    2bdc:	00848533          	add	a0,s1,s0
    2be0:	01813083          	ld	ra,24(sp)
    2be4:	01013403          	ld	s0,16(sp)
    2be8:	00813483          	ld	s1,8(sp)
    2bec:	02010113          	addi	sp,sp,32
    2bf0:	00008067          	ret

0000000000002bf4 <__GI_strchr>:
    2bf4:	0ff5f713          	zext.b	a4,a1

0000000000002bf8 <.L2>:
    2bf8:	00757793          	andi	a5,a0,7
    2bfc:	06079c63          	bnez	a5,2c74 <.L4>
    2c00:	0ff5f593          	zext.b	a1,a1
    2c04:	00859693          	slli	a3,a1,0x8
    2c08:	00b6e6b3          	or	a3,a3,a1
    2c0c:	01069793          	slli	a5,a3,0x10
    2c10:	00d7e7b3          	or	a5,a5,a3
    2c14:	02079613          	slli	a2,a5,0x20
    2c18:	00f66633          	or	a2,a2,a5
    2c1c:	00004597          	auipc	a1,0x4
    2c20:	d745b583          	ld	a1,-652(a1) # 6990 <.LC0>
    2c24:	00050793          	mv	a5,a0
    2c28:	fff00313          	li	t1,-1

0000000000002c2c <.L21>:
    2c2c:	00078513          	mv	a0,a5
    2c30:	0007b803          	ld	a6,0(a5)
    2c34:	00878793          	addi	a5,a5,8
    2c38:	010648b3          	xor	a7,a2,a6
    2c3c:	00b806b3          	add	a3,a6,a1
    2c40:	0106c6b3          	xor	a3,a3,a6
    2c44:	00b88833          	add	a6,a7,a1
    2c48:	01184833          	xor	a6,a6,a7
    2c4c:	0106f6b3          	and	a3,a3,a6
    2c50:	00b6e6b3          	or	a3,a3,a1
    2c54:	fc668ce3          	beq	a3,t1,2c2c <.L21>
    2c58:	ff87c683          	lbu	a3,-8(a5)
    2c5c:	0ae68663          	beq	a3,a4,2d08 <.L1>
    2c60:	0a068263          	beqz	a3,2d04 <.L20>
    2c64:	ff97c683          	lbu	a3,-7(a5)
    2c68:	02e69063          	bne	a3,a4,2c88 <.L6>
    2c6c:	00150513          	addi	a0,a0,1
    2c70:	00008067          	ret

0000000000002c74 <.L4>:
    2c74:	00054783          	lbu	a5,0(a0)
    2c78:	08e78863          	beq	a5,a4,2d08 <.L1>
    2c7c:	08078463          	beqz	a5,2d04 <.L20>
    2c80:	00150513          	addi	a0,a0,1
    2c84:	f75ff06f          	j	2bf8 <.L2>

0000000000002c88 <.L6>:
    2c88:	06068e63          	beqz	a3,2d04 <.L20>
    2c8c:	ffa7c683          	lbu	a3,-6(a5)
    2c90:	00e69663          	bne	a3,a4,2c9c <.L7>
    2c94:	00250513          	addi	a0,a0,2
    2c98:	00008067          	ret

0000000000002c9c <.L7>:
    2c9c:	06068463          	beqz	a3,2d04 <.L20>
    2ca0:	ffb7c683          	lbu	a3,-5(a5)
    2ca4:	00e69663          	bne	a3,a4,2cb0 <.L8>
    2ca8:	00350513          	addi	a0,a0,3
    2cac:	00008067          	ret

0000000000002cb0 <.L8>:
    2cb0:	04068a63          	beqz	a3,2d04 <.L20>
    2cb4:	ffc7c683          	lbu	a3,-4(a5)
    2cb8:	00e69663          	bne	a3,a4,2cc4 <.L9>
    2cbc:	00450513          	addi	a0,a0,4
    2cc0:	00008067          	ret

0000000000002cc4 <.L9>:
    2cc4:	04068063          	beqz	a3,2d04 <.L20>
    2cc8:	ffd7c683          	lbu	a3,-3(a5)
    2ccc:	00e69663          	bne	a3,a4,2cd8 <.L10>
    2cd0:	00550513          	addi	a0,a0,5
    2cd4:	00008067          	ret

0000000000002cd8 <.L10>:
    2cd8:	02068663          	beqz	a3,2d04 <.L20>
    2cdc:	ffe7c683          	lbu	a3,-2(a5)
    2ce0:	00e69663          	bne	a3,a4,2cec <.L11>
    2ce4:	00650513          	addi	a0,a0,6
    2ce8:	00008067          	ret

0000000000002cec <.L11>:
    2cec:	00068c63          	beqz	a3,2d04 <.L20>
    2cf0:	fff7c683          	lbu	a3,-1(a5)
    2cf4:	00e69663          	bne	a3,a4,2d00 <.L12>
    2cf8:	00750513          	addi	a0,a0,7
    2cfc:	00008067          	ret

0000000000002d00 <.L12>:
    2d00:	f20696e3          	bnez	a3,2c2c <.L21>

0000000000002d04 <.L20>:
    2d04:	00000513          	li	a0,0

0000000000002d08 <.L1>:
    2d08:	00008067          	ret

0000000000002d0c <__GI___xpg_strerror_r>:
    2d0c:	f9010113          	addi	sp,sp,-112
    2d10:	05313423          	sd	s3,72(sp)
    2d14:	05413023          	sd	s4,64(sp)
    2d18:	06113423          	sd	ra,104(sp)
    2d1c:	06813023          	sd	s0,96(sp)
    2d20:	04913c23          	sd	s1,88(sp)
    2d24:	05213823          	sd	s2,80(sp)
    2d28:	07c00793          	li	a5,124
    2d2c:	00058993          	mv	s3,a1
    2d30:	00060a13          	mv	s4,a2
    2d34:	08a7f263          	bgeu	a5,a0,2db8 <.L13>

0000000000002d38 <.L3>:
    2d38:	00050593          	mv	a1,a0
    2d3c:	00000693          	li	a3,0
    2d40:	ff600613          	li	a2,-10
    2d44:	03910513          	addi	a0,sp,57
    2d48:	a64ff0ef          	jal	ra,1fac <_uintmaxtostr>
    2d4c:	ff250493          	addi	s1,a0,-14
    2d50:	00e00613          	li	a2,14
    2d54:	00004597          	auipc	a1,0x4
    2d58:	c4458593          	addi	a1,a1,-956 # 6998 <unknown.0>
    2d5c:	00048513          	mv	a0,s1
    2d60:	ebcfe0ef          	jal	ra,141c <__GI_memcpy>
    2d64:	01600413          	li	s0,22
    2d68:	01c0006f          	j	2d84 <.L6>

0000000000002d6c <.L5>:
    2d6c:	00079463          	bnez	a5,2d74 <.L4>
    2d70:	fff4041b          	addiw	s0,s0,-1

0000000000002d74 <.L4>:
    2d74:	00148493          	addi	s1,s1,1

0000000000002d78 <.L2>:
    2d78:	0004c783          	lbu	a5,0(s1)
    2d7c:	fe0418e3          	bnez	s0,2d6c <.L5>
    2d80:	fa078ce3          	beqz	a5,2d38 <.L3>

0000000000002d84 <.L6>:
    2d84:	00099463          	bnez	s3,2d8c <.L7>
    2d88:	00000a13          	li	s4,0

0000000000002d8c <.L7>:
    2d8c:	00048513          	mv	a0,s1
    2d90:	da4fe0ef          	jal	ra,1334 <__GI_strlen>
    2d94:	0015091b          	addiw	s2,a0,1
    2d98:	032a7863          	bgeu	s4,s2,2dc8 <.L8>
    2d9c:	000a091b          	sext.w	s2,s4
    2da0:	02200413          	li	s0,34
    2da4:	02091463          	bnez	s2,2dcc <.L9>

0000000000002da8 <.L10>:
    2da8:	00003797          	auipc	a5,0x3
    2dac:	8807b783          	ld	a5,-1920(a5) # 5628 <_GLOBAL_OFFSET_TABLE_+0x18>
    2db0:	0087a023          	sw	s0,0(a5)
    2db4:	0340006f          	j	2de8 <.L12>

0000000000002db8 <.L13>:
    2db8:	00050413          	mv	s0,a0
    2dbc:	00003497          	auipc	s1,0x3
    2dc0:	e7c48493          	addi	s1,s1,-388 # 5c38 <_string_syserrmsgs>
    2dc4:	fb5ff06f          	j	2d78 <.L2>

0000000000002dc8 <.L8>:
    2dc8:	00090e63          	beqz	s2,2de4 <.L11>

0000000000002dcc <.L9>:
    2dcc:	00098513          	mv	a0,s3
    2dd0:	00090613          	mv	a2,s2
    2dd4:	00048593          	mv	a1,s1
    2dd8:	012989b3          	add	s3,s3,s2
    2ddc:	e40fe0ef          	jal	ra,141c <__GI_memcpy>
    2de0:	fe098fa3          	sb	zero,-1(s3)

0000000000002de4 <.L11>:
    2de4:	fc0412e3          	bnez	s0,2da8 <.L10>

0000000000002de8 <.L12>:
    2de8:	06813083          	ld	ra,104(sp)
    2dec:	00040513          	mv	a0,s0
    2df0:	06013403          	ld	s0,96(sp)
    2df4:	05813483          	ld	s1,88(sp)
    2df8:	05013903          	ld	s2,80(sp)
    2dfc:	04813983          	ld	s3,72(sp)
    2e00:	04013a03          	ld	s4,64(sp)
    2e04:	07010113          	addi	sp,sp,112
    2e08:	00008067          	ret

0000000000002e0c <__GI_exit>:
    2e0c:	ff010113          	addi	sp,sp,-16
    2e10:	00813023          	sd	s0,0(sp)
    2e14:	00006717          	auipc	a4,0x6
    2e18:	d4c73703          	ld	a4,-692(a4) # 8b60 <__exit_cleanup>
    2e1c:	00113423          	sd	ra,8(sp)
    2e20:	00070793          	mv	a5,a4
    2e24:	00050413          	mv	s0,a0
    2e28:	00078463          	beqz	a5,2e30 <.L2>
    2e2c:	000700e7          	jalr	a4

0000000000002e30 <.L2>:
    2e30:	b39fe0ef          	jal	ra,1968 <__uClibc_fini>
    2e34:	00003717          	auipc	a4,0x3
    2e38:	84c73703          	ld	a4,-1972(a4) # 5680 <.LLST14+0x34>
    2e3c:	00070793          	mv	a5,a4
    2e40:	00078463          	beqz	a5,2e48 <.L3>
    2e44:	000700e7          	jalr	a4

0000000000002e48 <.L3>:
    2e48:	00040513          	mv	a0,s0
    2e4c:	02c000ef          	jal	ra,2e78 <__GI__exit>

0000000000002e50 <__GI_write>:
    2e50:	04000893          	li	a7,64
    2e54:	00000073          	ecall
    2e58:	fffff7b7          	lui	a5,0xfffff
    2e5c:	00a7fc63          	bgeu	a5,a0,2e74 <.L1>
    2e60:	40a0053b          	negw	a0,a0
    2e64:	00002797          	auipc	a5,0x2
    2e68:	7c47b783          	ld	a5,1988(a5) # 5628 <_GLOBAL_OFFSET_TABLE_+0x18>
    2e6c:	00a7a023          	sw	a0,0(a5)
    2e70:	fff00513          	li	a0,-1

0000000000002e74 <.L1>:
    2e74:	00008067          	ret

0000000000002e78 <__GI__exit>:
    2e78:	00050713          	mv	a4,a0
    2e7c:	fffff637          	lui	a2,0xfffff
    2e80:	00002697          	auipc	a3,0x2
    2e84:	7a86b683          	ld	a3,1960(a3) # 5628 <_GLOBAL_OFFSET_TABLE_+0x18>

0000000000002e88 <.L2>:
    2e88:	00070513          	mv	a0,a4
    2e8c:	05d00893          	li	a7,93
    2e90:	00000073          	ecall
    2e94:	fea67ae3          	bgeu	a2,a0,2e88 <.L2>
    2e98:	40a007bb          	negw	a5,a0
    2e9c:	00f6a023          	sw	a5,0(a3)
    2ea0:	fe9ff06f          	j	2e88 <.L2>

0000000000002ea4 <__GI_fseek>:
    2ea4:	0040006f          	j	2ea8 <__GI_fseeko64>

0000000000002ea8 <__GI_fseeko64>:
    2ea8:	fd010113          	addi	sp,sp,-48
    2eac:	02113423          	sd	ra,40(sp)
    2eb0:	02813023          	sd	s0,32(sp)
    2eb4:	00b13c23          	sd	a1,24(sp)
    2eb8:	00200793          	li	a5,2
    2ebc:	02c7f463          	bgeu	a5,a2,2ee4 <.L2>
    2ec0:	00002797          	auipc	a5,0x2
    2ec4:	7687b783          	ld	a5,1896(a5) # 5628 <_GLOBAL_OFFSET_TABLE_+0x18>
    2ec8:	01600713          	li	a4,22
    2ecc:	00e7a023          	sw	a4,0(a5)

0000000000002ed0 <.L10>:
    2ed0:	fff00513          	li	a0,-1

0000000000002ed4 <.L3>:
    2ed4:	02813083          	ld	ra,40(sp)
    2ed8:	02013403          	ld	s0,32(sp)
    2edc:	03010113          	addi	sp,sp,48
    2ee0:	00008067          	ret

0000000000002ee4 <.L2>:
    2ee4:	00055783          	lhu	a5,0(a0)
    2ee8:	00050413          	mv	s0,a0
    2eec:	0407f793          	andi	a5,a5,64
    2ef0:	06079863          	bnez	a5,2f60 <.L4>

0000000000002ef4 <.L7>:
    2ef4:	00100793          	li	a5,1
    2ef8:	00f61e63          	bne	a2,a5,2f14 <.L5>
    2efc:	01810593          	addi	a1,sp,24
    2f00:	00040513          	mv	a0,s0
    2f04:	00c13423          	sd	a2,8(sp)
    2f08:	07c000ef          	jal	ra,2f84 <__stdio_adjust_position>
    2f0c:	00813603          	ld	a2,8(sp)
    2f10:	fc0540e3          	bltz	a0,2ed0 <.L10>

0000000000002f14 <.L5>:
    2f14:	00442703          	lw	a4,4(s0)
    2f18:	ffe00793          	li	a5,-2
    2f1c:	04f71c63          	bne	a4,a5,2f74 <.L19>
    2f20:	05843783          	ld	a5,88(s0)
    2f24:	fa0786e3          	beqz	a5,2ed0 <.L10>
    2f28:	04043503          	ld	a0,64(s0)
    2f2c:	01810593          	addi	a1,sp,24
    2f30:	000780e7          	jalr	a5

0000000000002f34 <.L11>:
    2f34:	f8054ee3          	bltz	a0,2ed0 <.L10>
    2f38:	00045783          	lhu	a5,0(s0)
    2f3c:	00000513          	li	a0,0
    2f40:	fb87f793          	andi	a5,a5,-72
    2f44:	00f41023          	sh	a5,0(s0)
    2f48:	00843783          	ld	a5,8(s0)
    2f4c:	00f43c23          	sd	a5,24(s0)
    2f50:	02f43023          	sd	a5,32(s0)
    2f54:	02f43423          	sd	a5,40(s0)
    2f58:	02f43823          	sd	a5,48(s0)
    2f5c:	f79ff06f          	j	2ed4 <.L3>

0000000000002f60 <.L4>:
    2f60:	00c13423          	sd	a2,8(sp)
    2f64:	ae8fd0ef          	jal	ra,24c <__stdio_wcommit>
    2f68:	00813603          	ld	a2,8(sp)
    2f6c:	f80504e3          	beqz	a0,2ef4 <.L7>
    2f70:	f61ff06f          	j	2ed0 <.L10>

0000000000002f74 <.L19>:
    2f74:	01810593          	addi	a1,sp,24
    2f78:	00040513          	mv	a0,s0
    2f7c:	068000ef          	jal	ra,2fe4 <__stdio_seek>
    2f80:	fb5ff06f          	j	2f34 <.L11>

0000000000002f84 <__stdio_adjust_position>:
    2f84:	00055783          	lhu	a5,0(a0)
    2f88:	00000713          	li	a4,0
    2f8c:	0037f693          	andi	a3,a5,3
    2f90:	00068463          	beqz	a3,2f98 <.L2>
    2f94:	fff6871b          	addiw	a4,a3,-1

0000000000002f98 <.L2>:
    2f98:	0407f793          	andi	a5,a5,64
    2f9c:	04078063          	beqz	a5,2fdc <.L3>
    2fa0:	00853783          	ld	a5,8(a0)

0000000000002fa4 <.L4>:
    2fa4:	01853683          	ld	a3,24(a0)
    2fa8:	40d787b3          	sub	a5,a5,a3
    2fac:	00e7853b          	addw	a0,a5,a4
    2fb0:	0005b703          	ld	a4,0(a1)
    2fb4:	40a706b3          	sub	a3,a4,a0
    2fb8:	00d5b023          	sd	a3,0(a1)
    2fbc:	00d75463          	bge	a4,a3,2fc4 <.L5>
    2fc0:	40a0053b          	negw	a0,a0

0000000000002fc4 <.L5>:
    2fc4:	00055a63          	bgez	a0,2fd8 <.L6>
    2fc8:	00002797          	auipc	a5,0x2
    2fcc:	6607b783          	ld	a5,1632(a5) # 5628 <_GLOBAL_OFFSET_TABLE_+0x18>
    2fd0:	04b00713          	li	a4,75
    2fd4:	00e7a023          	sw	a4,0(a5)

0000000000002fd8 <.L6>:
    2fd8:	00008067          	ret

0000000000002fdc <.L3>:
    2fdc:	02053783          	ld	a5,32(a0)
    2fe0:	fc5ff06f          	j	2fa4 <.L4>

0000000000002fe4 <__stdio_seek>:
    2fe4:	ff010113          	addi	sp,sp,-16
    2fe8:	00813023          	sd	s0,0(sp)
    2fec:	00452503          	lw	a0,4(a0)
    2ff0:	00058413          	mv	s0,a1
    2ff4:	0005b583          	ld	a1,0(a1)
    2ff8:	00113423          	sd	ra,8(sp)
    2ffc:	028000ef          	jal	ra,3024 <__GI_lseek>
    3000:	00054e63          	bltz	a0,301c <.L2>
    3004:	00a43023          	sd	a0,0(s0)
    3008:	00000513          	li	a0,0

000000000000300c <.L3>:
    300c:	00813083          	ld	ra,8(sp)
    3010:	00013403          	ld	s0,0(sp)
    3014:	01010113          	addi	sp,sp,16
    3018:	00008067          	ret

000000000000301c <.L2>:
    301c:	0005051b          	sext.w	a0,a0
    3020:	fedff06f          	j	300c <.L3>

0000000000003024 <__GI_lseek>:
    3024:	03e00893          	li	a7,62
    3028:	00000073          	ecall
    302c:	fffff7b7          	lui	a5,0xfffff
    3030:	00a7fc63          	bgeu	a5,a0,3048 <.L1>
    3034:	40a0053b          	negw	a0,a0
    3038:	00002797          	auipc	a5,0x2
    303c:	5f07b783          	ld	a5,1520(a5) # 5628 <_GLOBAL_OFFSET_TABLE_+0x18>
    3040:	00a7a023          	sw	a0,0(a5)
    3044:	fff00513          	li	a0,-1

0000000000003048 <.L1>:
    3048:	00008067          	ret

000000000000304c <__divtf3>:
#include "soft-fp.h"
#include "quad.h"

TFtype
__divtf3 (TFtype a, TFtype b)
{
    304c:	fa010113          	addi	sp,sp,-96
    3050:	03513423          	sd	s5,40(sp)
    3054:	03613023          	sd	s6,32(sp)
    3058:	04113c23          	sd	ra,88(sp)
    305c:	04813823          	sd	s0,80(sp)
    3060:	04913423          	sd	s1,72(sp)
    3064:	05213023          	sd	s2,64(sp)
    3068:	03313c23          	sd	s3,56(sp)
    306c:	03413823          	sd	s4,48(sp)
    3070:	01713c23          	sd	s7,24(sp)
    3074:	01813823          	sd	s8,16(sp)
    3078:	01913423          	sd	s9,8(sp)
    307c:	00060b13          	mv	s6,a2
    3080:	00068a93          	mv	s5,a3
  FP_DECL_Q (A);
  FP_DECL_Q (B);
  FP_DECL_Q (R);
  TFtype r;

  FP_INIT_ROUNDMODE;
    3084:	002029f3          	frrm	s3

0000000000003088 <.LBB2>:
  FP_UNPACK_Q (A, a);
    3088:	000087b7          	lui	a5,0x8
    308c:	0305d713          	srli	a4,a1,0x30
    3090:	fff78793          	addi	a5,a5,-1 # 7fff <.LLST3+0xd81>
    3094:	01059413          	slli	s0,a1,0x10
    3098:	00f77733          	and	a4,a4,a5

000000000000309c <.LBE2>:
  FP_INIT_ROUNDMODE;
    309c:	0009899b          	sext.w	s3,s3

00000000000030a0 <.LBB3>:
  FP_UNPACK_Q (A, a);
    30a0:	01045413          	srli	s0,s0,0x10

00000000000030a4 <.LVL2>:
    30a4:	03f5da13          	srli	s4,a1,0x3f

00000000000030a8 <.LBE3>:
    30a8:	04070263          	beqz	a4,30ec <.L2>
    30ac:	00050493          	mv	s1,a0
    30b0:	0007081b          	sext.w	a6,a4
    30b4:	0af70663          	beq	a4,a5,3160 <.L3>

00000000000030b8 <.LVL4>:
    30b8:	03d55793          	srli	a5,a0,0x3d
    30bc:	00341413          	slli	s0,s0,0x3

00000000000030c0 <.LVL5>:
    30c0:	00100693          	li	a3,1
    30c4:	ffffc837          	lui	a6,0xffffc
    30c8:	0087e7b3          	or	a5,a5,s0
    30cc:	03369693          	slli	a3,a3,0x33
    30d0:	00180813          	addi	a6,a6,1 # ffffffffffffc001 <_ebss+0xffffffffffff33d1>
    30d4:	00d7ebb3          	or	s7,a5,a3

00000000000030d8 <.LVL6>:
    30d8:	00351493          	slli	s1,a0,0x3

00000000000030dc <.LVL7>:
    30dc:	01070c33          	add	s8,a4,a6

00000000000030e0 <.L145>:
    30e0:	00000c93          	li	s9,0

00000000000030e4 <.L135>:
  FP_DECL_EX;
    30e4:	00000913          	li	s2,0
    30e8:	0980006f          	j	3180 <.L4>

00000000000030ec <.L2>:
  FP_UNPACK_Q (A, a);
    30ec:	00856bb3          	or	s7,a0,s0
    30f0:	00050913          	mv	s2,a0
    30f4:	100b8e63          	beqz	s7,3210 <.L72>

00000000000030f8 <.LBB11>:
    30f8:	04040663          	beqz	s0,3144 <.L5>
    30fc:	00040513          	mv	a0,s0

0000000000003100 <.LVL11>:
    3100:	488020ef          	jal	ra,5588 <__clzdi2>

0000000000003104 <.LVL12>:
    3104:	0005071b          	sext.w	a4,a0

0000000000003108 <.L6>:
    3108:	ff170693          	addi	a3,a4,-15

000000000000310c <.LVL14>:
    310c:	03c00613          	li	a2,60
    3110:	0006879b          	sext.w	a5,a3
    3114:	02d64e63          	blt	a2,a3,3150 <.L7>
    3118:	03d00693          	li	a3,61

000000000000311c <.LVL15>:
    311c:	0037849b          	addiw	s1,a5,3
    3120:	40f687bb          	subw	a5,a3,a5
    3124:	00941433          	sll	s0,s0,s1

0000000000003128 <.LVL16>:
    3128:	00f957b3          	srl	a5,s2,a5
    312c:	0087ebb3          	or	s7,a5,s0

0000000000003130 <.LVL17>:
    3130:	009914b3          	sll	s1,s2,s1

0000000000003134 <.L8>:
    3134:	ffffc837          	lui	a6,0xffffc
    3138:	01180813          	addi	a6,a6,17 # ffffffffffffc011 <_ebss+0xffffffffffff33e1>
    313c:	40e80c33          	sub	s8,a6,a4
    3140:	fa1ff06f          	j	30e0 <.L145>

0000000000003144 <.L5>:
    3144:	444020ef          	jal	ra,5588 <__clzdi2>

0000000000003148 <.LVL20>:
    3148:	0405071b          	addiw	a4,a0,64

000000000000314c <.LVL21>:
    314c:	fbdff06f          	j	3108 <.L6>

0000000000003150 <.L7>:
    3150:	fc37879b          	addiw	a5,a5,-61
    3154:	00f91bb3          	sll	s7,s2,a5

0000000000003158 <.LVL23>:
    3158:	00000493          	li	s1,0
    315c:	fd9ff06f          	j	3134 <.L8>

0000000000003160 <.L3>:
    3160:	00856bb3          	or	s7,a0,s0
    3164:	0a0b8e63          	beqz	s7,3220 <.L73>

0000000000003168 <.LVL25>:
    3168:	02f45913          	srli	s2,s0,0x2f
    316c:	00194913          	xori	s2,s2,1
    3170:	00491913          	slli	s2,s2,0x4

0000000000003174 <.LBB12>:
    3174:	00040b93          	mv	s7,s0

0000000000003178 <.LBE12>:
    3178:	00080c13          	mv	s8,a6
    317c:	00300c93          	li	s9,3

0000000000003180 <.L4>:
  FP_UNPACK_Q (B, b);
    3180:	000086b7          	lui	a3,0x8
    3184:	030ad613          	srli	a2,s5,0x30
    3188:	fff68693          	addi	a3,a3,-1 # 7fff <.LLST3+0xd81>
    318c:	010a9413          	slli	s0,s5,0x10
    3190:	00d67633          	and	a2,a2,a3
    3194:	000b0793          	mv	a5,s6

0000000000003198 <.LVL27>:
    3198:	01045413          	srli	s0,s0,0x10

000000000000319c <.LVL28>:
    319c:	03fada93          	srli	s5,s5,0x3f

00000000000031a0 <.LBE13>:
    31a0:	08060863          	beqz	a2,3230 <.L9>
    31a4:	0006071b          	sext.w	a4,a2
    31a8:	0ed60e63          	beq	a2,a3,32a4 <.L10>

00000000000031ac <.LVL30>:
    31ac:	00341413          	slli	s0,s0,0x3

00000000000031b0 <.LVL31>:
    31b0:	03db5713          	srli	a4,s6,0x3d
    31b4:	00876733          	or	a4,a4,s0
    31b8:	00100413          	li	s0,1
    31bc:	03341413          	slli	s0,s0,0x33
    31c0:	00876433          	or	s0,a4,s0

00000000000031c4 <.LVL32>:
    31c4:	ffffc737          	lui	a4,0xffffc
    31c8:	00170713          	addi	a4,a4,1 # ffffffffffffc001 <_ebss+0xffffffffffff33d1>
    31cc:	003b1793          	slli	a5,s6,0x3
    31d0:	00e60733          	add	a4,a2,a4

00000000000031d4 <.L136>:
    31d4:	00000693          	li	a3,0

00000000000031d8 <.L11>:
  FP_DIV_Q (R, A, B);
    31d8:	40ec0833          	sub	a6,s8,a4
    31dc:	002c9713          	slli	a4,s9,0x2

00000000000031e0 <.LVL36>:
    31e0:	00d76733          	or	a4,a4,a3
    31e4:	fff70713          	addi	a4,a4,-1
    31e8:	00e00593          	li	a1,14
    31ec:	015a4633          	xor	a2,s4,s5

00000000000031f0 <.LVL37>:
    31f0:	12e5ea63          	bltu	a1,a4,3324 <.L16>
    31f4:	00003597          	auipc	a1,0x3
    31f8:	5a058593          	addi	a1,a1,1440 # 6794 <.L18>
    31fc:	00271713          	slli	a4,a4,0x2
    3200:	00b70733          	add	a4,a4,a1
    3204:	00072703          	lw	a4,0(a4)
    3208:	00b70733          	add	a4,a4,a1
    320c:	00070067          	jr	a4

0000000000003210 <.L72>:
  FP_UNPACK_Q (A, a);
    3210:	00000493          	li	s1,0

0000000000003214 <.LBE20>:
    3214:	00000c13          	li	s8,0
    3218:	00100c93          	li	s9,1
    321c:	ec9ff06f          	j	30e4 <.L135>

0000000000003220 <.L73>:
    3220:	00000493          	li	s1,0

0000000000003224 <.LBE21>:
    3224:	00080c13          	mv	s8,a6
    3228:	00200c93          	li	s9,2
    322c:	eb9ff06f          	j	30e4 <.L135>

0000000000003230 <.L9>:
  FP_UNPACK_Q (B, b);
    3230:	008b67b3          	or	a5,s6,s0

0000000000003234 <.LVL42>:
    3234:	08078863          	beqz	a5,32c4 <.L74>

0000000000003238 <.LBB22>:
    3238:	04040663          	beqz	s0,3284 <.L12>
    323c:	00040513          	mv	a0,s0
    3240:	348020ef          	jal	ra,5588 <__clzdi2>

0000000000003244 <.LVL43>:
    3244:	0005051b          	sext.w	a0,a0

0000000000003248 <.L13>:
    3248:	ff150793          	addi	a5,a0,-15

000000000000324c <.LVL45>:
    324c:	03c00693          	li	a3,60
    3250:	0007871b          	sext.w	a4,a5
    3254:	04f6c063          	blt	a3,a5,3294 <.L14>
    3258:	03d00693          	li	a3,61
    325c:	0037079b          	addiw	a5,a4,3

0000000000003260 <.LVL46>:
    3260:	40e6873b          	subw	a4,a3,a4
    3264:	00f41433          	sll	s0,s0,a5

0000000000003268 <.LVL47>:
    3268:	00eb5733          	srl	a4,s6,a4
    326c:	00876433          	or	s0,a4,s0

0000000000003270 <.LVL48>:
    3270:	00fb17b3          	sll	a5,s6,a5

0000000000003274 <.L15>:
    3274:	ffffc737          	lui	a4,0xffffc
    3278:	01170713          	addi	a4,a4,17 # ffffffffffffc011 <_ebss+0xffffffffffff33e1>
    327c:	40a70733          	sub	a4,a4,a0
    3280:	f55ff06f          	j	31d4 <.L136>

0000000000003284 <.L12>:
    3284:	000b0513          	mv	a0,s6
    3288:	300020ef          	jal	ra,5588 <__clzdi2>

000000000000328c <.LVL51>:
    328c:	0405051b          	addiw	a0,a0,64

0000000000003290 <.LVL52>:
    3290:	fb9ff06f          	j	3248 <.L13>

0000000000003294 <.L14>:
    3294:	fc37071b          	addiw	a4,a4,-61
    3298:	00eb1433          	sll	s0,s6,a4

000000000000329c <.LVL54>:
    329c:	00000793          	li	a5,0

00000000000032a0 <.LVL55>:
    32a0:	fd5ff06f          	j	3274 <.L15>

00000000000032a4 <.L10>:
    32a4:	008b66b3          	or	a3,s6,s0
    32a8:	02068863          	beqz	a3,32d8 <.L75>

00000000000032ac <.LVL57>:
    32ac:	02f45693          	srli	a3,s0,0x2f
    32b0:	0016f693          	andi	a3,a3,1
    32b4:	02069a63          	bnez	a3,32e8 <.L76>
    32b8:	00300693          	li	a3,3
    32bc:	01000913          	li	s2,16

00000000000032c0 <.LVL58>:
    32c0:	f19ff06f          	j	31d8 <.L11>

00000000000032c4 <.L74>:
    32c4:	00000413          	li	s0,0

00000000000032c8 <.LVL60>:
    32c8:	00000793          	li	a5,0

00000000000032cc <.LBE23>:
    32cc:	00000713          	li	a4,0
    32d0:	00100693          	li	a3,1
    32d4:	f05ff06f          	j	31d8 <.L11>

00000000000032d8 <.L75>:
    32d8:	00000413          	li	s0,0

00000000000032dc <.LVL62>:
    32dc:	00000793          	li	a5,0

00000000000032e0 <.LBE24>:
    32e0:	00200693          	li	a3,2
    32e4:	ef5ff06f          	j	31d8 <.L11>

00000000000032e8 <.L76>:
    32e8:	00300693          	li	a3,3
    32ec:	eedff06f          	j	31d8 <.L11>

00000000000032f0 <.L20>:
    32f0:	000a8613          	mv	a2,s5

00000000000032f4 <.L17>:
  FP_PACK_Q (r, R);
    32f4:	00300713          	li	a4,3
    32f8:	64e68c63          	beq	a3,a4,3950 <.L88>
    32fc:	00100713          	li	a4,1
    3300:	66e68663          	beq	a3,a4,396c <.L89>
    3304:	00200713          	li	a4,2
    3308:	2ce69863          	bne	a3,a4,35d8 <.L35>
    330c:	32c0006f          	j	3638 <.L90>

0000000000003310 <.L19>:
  FP_UNPACK_Q (A, a);
    3310:	000a0613          	mv	a2,s4
    3314:	000b8413          	mv	s0,s7

0000000000003318 <.LVL68>:
    3318:	00048793          	mv	a5,s1

000000000000331c <.LBE26>:
  FP_DIV_Q (R, A, B);
    331c:	000c8693          	mv	a3,s9

0000000000003320 <.LVL70>:
    3320:	fd5ff06f          	j	32f4 <.L17>

0000000000003324 <.L16>:
    3324:	01746663          	bltu	s0,s7,3330 <.L25>
    3328:	308b9063          	bne	s7,s0,3628 <.L26>
    332c:	2ef4ee63          	bltu	s1,a5,3628 <.L26>

0000000000003330 <.L25>:
    3330:	03fb9693          	slli	a3,s7,0x3f

0000000000003334 <.LVL73>:
    3334:	0014d713          	srli	a4,s1,0x1

0000000000003338 <.LVL74>:
    3338:	03f49e93          	slli	t4,s1,0x3f

000000000000333c <.LVL75>:
    333c:	001bdb93          	srli	s7,s7,0x1

0000000000003340 <.LVL76>:
    3340:	00e6e4b3          	or	s1,a3,a4

0000000000003344 <.L27>:
    3344:	00c41593          	slli	a1,s0,0xc
    3348:	0347d713          	srli	a4,a5,0x34
    334c:	00b76733          	or	a4,a4,a1

0000000000003350 <.LBB29>:
    3350:	0205d593          	srli	a1,a1,0x20
    3354:	02bbde33          	divu	t3,s7,a1

0000000000003358 <.LBE29>:
    3358:	00c79693          	slli	a3,a5,0xc

000000000000335c <.LBB31>:
    335c:	02071513          	slli	a0,a4,0x20
    3360:	02055513          	srli	a0,a0,0x20

0000000000003364 <.LVL80>:
    3364:	0204d893          	srli	a7,s1,0x20
    3368:	02bbf7b3          	remu	a5,s7,a1

000000000000336c <.LVL81>:
    336c:	03c50333          	mul	t1,a0,t3

0000000000003370 <.LVL82>:
    3370:	02079793          	slli	a5,a5,0x20

0000000000003374 <.LVL83>:
    3374:	00f8e7b3          	or	a5,a7,a5

0000000000003378 <.LVL84>:
    3378:	000e0893          	mv	a7,t3
    337c:	0067fe63          	bgeu	a5,t1,3398 <.L28>
    3380:	00e787b3          	add	a5,a5,a4

0000000000003384 <.LVL85>:
    3384:	fffe0893          	addi	a7,t3,-1

0000000000003388 <.LVL86>:
    3388:	00e7e863          	bltu	a5,a4,3398 <.L28>
    338c:	0067f663          	bgeu	a5,t1,3398 <.L28>
    3390:	ffee0893          	addi	a7,t3,-2
    3394:	00e787b3          	add	a5,a5,a4

0000000000003398 <.L28>:
    3398:	406787b3          	sub	a5,a5,t1

000000000000339c <.LVL89>:
    339c:	02b7de33          	divu	t3,a5,a1

00000000000033a0 <.LVL90>:
    33a0:	02049493          	slli	s1,s1,0x20

00000000000033a4 <.LVL91>:
    33a4:	0204d493          	srli	s1,s1,0x20
    33a8:	02b7f7b3          	remu	a5,a5,a1

00000000000033ac <.LVL92>:
    33ac:	03c50333          	mul	t1,a0,t3
    33b0:	02079793          	slli	a5,a5,0x20

00000000000033b4 <.LVL94>:
    33b4:	00f4e4b3          	or	s1,s1,a5

00000000000033b8 <.LVL95>:
    33b8:	000e0793          	mv	a5,t3
    33bc:	0064fe63          	bgeu	s1,t1,33d8 <.L29>
    33c0:	00e484b3          	add	s1,s1,a4

00000000000033c4 <.LVL96>:
    33c4:	fffe0793          	addi	a5,t3,-1

00000000000033c8 <.LVL97>:
    33c8:	00e4e863          	bltu	s1,a4,33d8 <.L29>
    33cc:	0064f663          	bgeu	s1,t1,33d8 <.L29>
    33d0:	ffee0793          	addi	a5,t3,-2
    33d4:	00e484b3          	add	s1,s1,a4

00000000000033d8 <.L29>:
    33d8:	406484b3          	sub	s1,s1,t1
    33dc:	02089893          	slli	a7,a7,0x20

00000000000033e0 <.LVL101>:
    33e0:	fff00313          	li	t1,-1

00000000000033e4 <.LVL102>:
    33e4:	00f8e8b3          	or	a7,a7,a5

00000000000033e8 <.LBB32>:
    33e8:	02035313          	srli	t1,t1,0x20
    33ec:	0068f7b3          	and	a5,a7,t1

00000000000033f0 <.LVL104>:
    33f0:	0208df13          	srli	t5,a7,0x20

00000000000033f4 <.LVL105>:
    33f4:	0206de13          	srli	t3,a3,0x20

00000000000033f8 <.LVL106>:
    33f8:	0066f333          	and	t1,a3,t1
    33fc:	026782b3          	mul	t0,a5,t1

0000000000003400 <.LVL107>:
    3400:	026f03b3          	mul	t2,t5,t1

0000000000003404 <.LVL108>:
    3404:	02fe07b3          	mul	a5,t3,a5

0000000000003408 <.LVL109>:
    3408:	00778fb3          	add	t6,a5,t2
    340c:	0202d793          	srli	a5,t0,0x20

0000000000003410 <.LVL110>:
    3410:	01f787b3          	add	a5,a5,t6
    3414:	03cf0f33          	mul	t5,t5,t3

0000000000003418 <.LVL111>:
    3418:	0077f863          	bgeu	a5,t2,3428 <.L30>
    341c:	00100f93          	li	t6,1
    3420:	020f9f93          	slli	t6,t6,0x20
    3424:	01ff0f33          	add	t5,t5,t6

0000000000003428 <.L30>:
    3428:	0207df93          	srli	t6,a5,0x20
    342c:	01ef8fb3          	add	t6,t6,t5

0000000000003430 <.LVL113>:
    3430:	fff00f13          	li	t5,-1

0000000000003434 <.LVL114>:
    3434:	020f5f13          	srli	t5,t5,0x20
    3438:	01e7f7b3          	and	a5,a5,t5

000000000000343c <.LVL115>:
    343c:	02079793          	slli	a5,a5,0x20
    3440:	01e2f2b3          	and	t0,t0,t5

0000000000003444 <.LVL116>:
    3444:	005787b3          	add	a5,a5,t0

0000000000003448 <.LBE32>:
    3448:	01f4e863          	bltu	s1,t6,3458 <.L31>

000000000000344c <.LBB33>:
    344c:	00088413          	mv	s0,a7

0000000000003450 <.LBE33>:
    3450:	05f49463          	bne	s1,t6,3498 <.L32>

0000000000003454 <.LVL118>:
    3454:	04fef263          	bgeu	t4,a5,3498 <.L32>

0000000000003458 <.L31>:
    3458:	00de8eb3          	add	t4,t4,a3

000000000000345c <.LVL120>:
    345c:	00debf33          	sltu	t5,t4,a3
    3460:	00ef0f33          	add	t5,t5,a4
    3464:	01e484b3          	add	s1,s1,t5

0000000000003468 <.LBE34>:
    3468:	fff88413          	addi	s0,a7,-1

000000000000346c <.LBB35>:
    346c:	00976663          	bltu	a4,s1,3478 <.L33>
    3470:	02971463          	bne	a4,s1,3498 <.L32>
    3474:	02dee263          	bltu	t4,a3,3498 <.L32>

0000000000003478 <.L33>:
    3478:	01f4e663          	bltu	s1,t6,3484 <.L34>
    347c:	009f9e63          	bne	t6,s1,3498 <.L32>
    3480:	00fefc63          	bgeu	t4,a5,3498 <.L32>

0000000000003484 <.L34>:
    3484:	00de8eb3          	add	t4,t4,a3

0000000000003488 <.LBE36>:
    3488:	ffe88413          	addi	s0,a7,-2

000000000000348c <.LBB37>:
    348c:	00deb8b3          	sltu	a7,t4,a3

0000000000003490 <.LVL125>:
    3490:	00e888b3          	add	a7,a7,a4
    3494:	011484b3          	add	s1,s1,a7

0000000000003498 <.L32>:
    3498:	40fe8f33          	sub	t5,t4,a5

000000000000349c <.LVL127>:
    349c:	41f484b3          	sub	s1,s1,t6

00000000000034a0 <.LVL128>:
    34a0:	01eeb8b3          	sltu	a7,t4,t5
    34a4:	411484b3          	sub	s1,s1,a7

00000000000034a8 <.LBE38>:
    34a8:	fff00793          	li	a5,-1

00000000000034ac <.LVL130>:
    34ac:	12970663          	beq	a4,s1,35d8 <.L35>

00000000000034b0 <.LBB39>:
    34b0:	02b4dfb3          	divu	t6,s1,a1

00000000000034b4 <.LVL132>:
    34b4:	020f5793          	srli	a5,t5,0x20
    34b8:	02b4f4b3          	remu	s1,s1,a1

00000000000034bc <.LVL133>:
    34bc:	000f8893          	mv	a7,t6
    34c0:	03f50eb3          	mul	t4,a0,t6

00000000000034c4 <.LVL134>:
    34c4:	02049493          	slli	s1,s1,0x20

00000000000034c8 <.LVL135>:
    34c8:	0097e7b3          	or	a5,a5,s1

00000000000034cc <.LVL136>:
    34cc:	01d7fe63          	bgeu	a5,t4,34e8 <.L36>
    34d0:	00e787b3          	add	a5,a5,a4

00000000000034d4 <.LVL137>:
    34d4:	ffff8893          	addi	a7,t6,-1

00000000000034d8 <.LVL138>:
    34d8:	00e7e863          	bltu	a5,a4,34e8 <.L36>
    34dc:	01d7f663          	bgeu	a5,t4,34e8 <.L36>
    34e0:	ffef8893          	addi	a7,t6,-2
    34e4:	00e787b3          	add	a5,a5,a4

00000000000034e8 <.L36>:
    34e8:	41d787b3          	sub	a5,a5,t4

00000000000034ec <.LVL141>:
    34ec:	02b7deb3          	divu	t4,a5,a1

00000000000034f0 <.LVL142>:
    34f0:	02b7f7b3          	remu	a5,a5,a1

00000000000034f4 <.LVL143>:
    34f4:	020f1593          	slli	a1,t5,0x20

00000000000034f8 <.LVL144>:
    34f8:	0205d593          	srli	a1,a1,0x20
    34fc:	03d50533          	mul	a0,a0,t4

0000000000003500 <.LVL145>:
    3500:	02079793          	slli	a5,a5,0x20

0000000000003504 <.LVL146>:
    3504:	00f5e7b3          	or	a5,a1,a5

0000000000003508 <.LVL147>:
    3508:	000e8593          	mv	a1,t4
    350c:	00a7fe63          	bgeu	a5,a0,3528 <.L37>
    3510:	00e787b3          	add	a5,a5,a4

0000000000003514 <.LVL148>:
    3514:	fffe8593          	addi	a1,t4,-1

0000000000003518 <.LVL149>:
    3518:	00e7e863          	bltu	a5,a4,3528 <.L37>
    351c:	00a7f663          	bgeu	a5,a0,3528 <.L37>
    3520:	ffee8593          	addi	a1,t4,-2
    3524:	00e787b3          	add	a5,a5,a4

0000000000003528 <.L37>:
    3528:	02089893          	slli	a7,a7,0x20

000000000000352c <.LVL152>:
    352c:	00b8e8b3          	or	a7,a7,a1
    3530:	02089593          	slli	a1,a7,0x20

0000000000003534 <.LVL153>:
    3534:	0205d593          	srli	a1,a1,0x20
    3538:	40a78533          	sub	a0,a5,a0

000000000000353c <.LBB40>:
    353c:	0208d793          	srli	a5,a7,0x20

0000000000003540 <.LVL155>:
    3540:	02b30eb3          	mul	t4,t1,a1

0000000000003544 <.LVL156>:
    3544:	02678333          	mul	t1,a5,t1

0000000000003548 <.LVL157>:
    3548:	02fe07b3          	mul	a5,t3,a5

000000000000354c <.LVL158>:
    354c:	02be0e33          	mul	t3,t3,a1

0000000000003550 <.LVL159>:
    3550:	020ed593          	srli	a1,t4,0x20
    3554:	006e0e33          	add	t3,t3,t1

0000000000003558 <.LVL160>:
    3558:	01c585b3          	add	a1,a1,t3

000000000000355c <.LVL161>:
    355c:	0065f863          	bgeu	a1,t1,356c <.L38>
    3560:	00100313          	li	t1,1

0000000000003564 <.LVL162>:
    3564:	02031313          	slli	t1,t1,0x20
    3568:	006787b3          	add	a5,a5,t1

000000000000356c <.L38>:
    356c:	0205d313          	srli	t1,a1,0x20
    3570:	00f30333          	add	t1,t1,a5

0000000000003574 <.LVL164>:
    3574:	fff00793          	li	a5,-1

0000000000003578 <.LVL165>:
    3578:	0207d793          	srli	a5,a5,0x20

000000000000357c <.LVL166>:
    357c:	00f5f5b3          	and	a1,a1,a5

0000000000003580 <.LVL167>:
    3580:	02059593          	slli	a1,a1,0x20
    3584:	00fefeb3          	and	t4,t4,a5

0000000000003588 <.LVL168>:
    3588:	01d585b3          	add	a1,a1,t4

000000000000358c <.LBE40>:
    358c:	00656863          	bltu	a0,t1,359c <.L39>

0000000000003590 <.LBB41>:
    3590:	00088793          	mv	a5,a7

0000000000003594 <.LBE41>:
    3594:	04651063          	bne	a0,t1,35d4 <.L44>

0000000000003598 <.LVL170>:
    3598:	04058063          	beqz	a1,35d8 <.L35>

000000000000359c <.L39>:
    359c:	00a70533          	add	a0,a4,a0

00000000000035a0 <.LBE42>:
    35a0:	fff88793          	addi	a5,a7,-1

00000000000035a4 <.LBB43>:
    35a4:	02e56463          	bltu	a0,a4,35cc <.L41>
    35a8:	00656663          	bltu	a0,t1,35b4 <.L42>
    35ac:	02651463          	bne	a0,t1,35d4 <.L44>
    35b0:	02b6f063          	bgeu	a3,a1,35d0 <.L43>

00000000000035b4 <.L42>:
    35b4:	ffe88793          	addi	a5,a7,-2

00000000000035b8 <.LBB44>:
    35b8:	00169893          	slli	a7,a3,0x1

00000000000035bc <.LVL175>:
    35bc:	00d8b6b3          	sltu	a3,a7,a3

00000000000035c0 <.LVL176>:
    35c0:	00e686b3          	add	a3,a3,a4
    35c4:	00d50533          	add	a0,a0,a3

00000000000035c8 <.LVL177>:
    35c8:	00088693          	mv	a3,a7

00000000000035cc <.L41>:
    35cc:	00651463          	bne	a0,t1,35d4 <.L44>

00000000000035d0 <.L43>:
    35d0:	00d58463          	beq	a1,a3,35d8 <.L35>

00000000000035d4 <.L44>:
    35d4:	0017e793          	ori	a5,a5,1

00000000000035d8 <.L35>:
  FP_PACK_Q (r, R);
    35d8:	00004737          	lui	a4,0x4
    35dc:	fff70713          	addi	a4,a4,-1 # 3fff <.L29+0x7>
    35e0:	00e80733          	add	a4,a6,a4

00000000000035e4 <.LVL182>:
    35e4:	16e05c63          	blez	a4,375c <.L45>
    35e8:	0077f693          	andi	a3,a5,7
    35ec:	08068463          	beqz	a3,3674 <.L46>
    35f0:	00200693          	li	a3,2
    35f4:	00196913          	ori	s2,s2,1
    35f8:	06d98c63          	beq	s3,a3,3670 <.L47>
    35fc:	00300693          	li	a3,3
    3600:	06d98263          	beq	s3,a3,3664 <.L48>
    3604:	06099863          	bnez	s3,3674 <.L46>
    3608:	00f7f693          	andi	a3,a5,15
    360c:	00400593          	li	a1,4
    3610:	06b68263          	beq	a3,a1,3674 <.L46>

0000000000003614 <.LBB45>:
    3614:	00478693          	addi	a3,a5,4

0000000000003618 <.L137>:
    3618:	00f6b7b3          	sltu	a5,a3,a5

000000000000361c <.LVL185>:
    361c:	00f40433          	add	s0,s0,a5

0000000000003620 <.LVL186>:
    3620:	00068793          	mv	a5,a3
    3624:	0500006f          	j	3674 <.L46>

0000000000003628 <.L26>:
  FP_DIV_Q (R, A, B);
    3628:	fff80813          	addi	a6,a6,-1

000000000000362c <.LVL188>:
    362c:	00000e93          	li	t4,0
    3630:	d15ff06f          	j	3344 <.L27>

0000000000003634 <.L24>:
    3634:	00896913          	ori	s2,s2,8

0000000000003638 <.L90>:
  FP_PACK_Q (r, R);
    3638:	00008737          	lui	a4,0x8
    363c:	00000593          	li	a1,0
    3640:	00000513          	li	a0,0
    3644:	fff70713          	addi	a4,a4,-1 # 7fff <.LLST3+0xd81>
    3648:	06c0006f          	j	36b4 <.L23>

000000000000364c <.L78>:
  FP_DIV_Q (R, A, B);
    364c:	00100413          	li	s0,1

0000000000003650 <.LVL192>:
    3650:	02f41413          	slli	s0,s0,0x2f
    3654:	00000793          	li	a5,0

0000000000003658 <.LVL193>:
    3658:	00000613          	li	a2,0
    365c:	00300693          	li	a3,3

0000000000003660 <.LVL194>:
    3660:	c95ff06f          	j	32f4 <.L17>

0000000000003664 <.L48>:
  FP_PACK_Q (r, R);
    3664:	00061863          	bnez	a2,3674 <.L46>

0000000000003668 <.L138>:
    3668:	00878693          	addi	a3,a5,8
    366c:	fadff06f          	j	3618 <.L137>

0000000000003670 <.L47>:
    3670:	fe061ce3          	bnez	a2,3668 <.L138>

0000000000003674 <.L46>:
    3674:	03445693          	srli	a3,s0,0x34
    3678:	0016f693          	andi	a3,a3,1
    367c:	00068e63          	beqz	a3,3698 <.L49>
    3680:	fff00713          	li	a4,-1

0000000000003684 <.LVL196>:
    3684:	03471713          	slli	a4,a4,0x34
    3688:	fff70713          	addi	a4,a4,-1
    368c:	00e47433          	and	s0,s0,a4
    3690:	00004737          	lui	a4,0x4

0000000000003694 <.LVL198>:
    3694:	00e80733          	add	a4,a6,a4

0000000000003698 <.L49>:
    3698:	000086b7          	lui	a3,0x8
    369c:	ffe68593          	addi	a1,a3,-2 # 7ffe <.LLST3+0xd80>
    36a0:	06e5c863          	blt	a1,a4,3710 <.L50>
    36a4:	0037d793          	srli	a5,a5,0x3

00000000000036a8 <.LVL200>:
    36a8:	03d41513          	slli	a0,s0,0x3d

00000000000036ac <.LVL201>:
    36ac:	00f56533          	or	a0,a0,a5

00000000000036b0 <.LVL202>:
    36b0:	00345593          	srli	a1,s0,0x3

00000000000036b4 <.L23>:
    36b4:	03171713          	slli	a4,a4,0x31
    36b8:	03175713          	srli	a4,a4,0x31
    36bc:	00f61613          	slli	a2,a2,0xf
    36c0:	01059593          	slli	a1,a1,0x10
    36c4:	00e66633          	or	a2,a2,a4
    36c8:	03061613          	slli	a2,a2,0x30
    36cc:	0105d593          	srli	a1,a1,0x10
    36d0:	00c5e5b3          	or	a1,a1,a2

00000000000036d4 <.LBE51>:
  FP_HANDLE_EXCEPTIONS;
    36d4:	00090463          	beqz	s2,36dc <.L1>
    36d8:	00192073          	csrs	fflags,s2

00000000000036dc <.L1>:

  return r;
}
    36dc:	05813083          	ld	ra,88(sp)
    36e0:	05013403          	ld	s0,80(sp)
    36e4:	04813483          	ld	s1,72(sp)
    36e8:	04013903          	ld	s2,64(sp)

00000000000036ec <.LVL205>:
    36ec:	03813983          	ld	s3,56(sp)

00000000000036f0 <.LVL206>:
    36f0:	03013a03          	ld	s4,48(sp)

00000000000036f4 <.LVL207>:
    36f4:	02813a83          	ld	s5,40(sp)

00000000000036f8 <.LVL208>:
    36f8:	02013b03          	ld	s6,32(sp)
    36fc:	01813b83          	ld	s7,24(sp)
    3700:	01013c03          	ld	s8,16(sp)

0000000000003704 <.LVL209>:
    3704:	00813c83          	ld	s9,8(sp)

0000000000003708 <.LVL210>:
    3708:	06010113          	addi	sp,sp,96
    370c:	00008067          	ret

0000000000003710 <.L50>:
  FP_PACK_Q (r, R);
    3710:	00200793          	li	a5,2

0000000000003714 <.LVL212>:
    3714:	02f98c63          	beq	s3,a5,374c <.L51>
    3718:	00300793          	li	a5,3
    371c:	fff68693          	addi	a3,a3,-1
    3720:	00f98a63          	beq	s3,a5,3734 <.L52>
    3724:	00099a63          	bnez	s3,3738 <.L91>

0000000000003728 <.L92>:
    3728:	00000513          	li	a0,0

000000000000372c <.LVL213>:
    372c:	00068713          	mv	a4,a3
    3730:	0100006f          	j	3740 <.L53>

0000000000003734 <.L52>:
    3734:	fe060ae3          	beqz	a2,3728 <.L92>

0000000000003738 <.L91>:
    3738:	fff00513          	li	a0,-1

000000000000373c <.LVL215>:
    373c:	00058713          	mv	a4,a1

0000000000003740 <.L53>:
    3740:	00596913          	ori	s2,s2,5

0000000000003744 <.LVL217>:
    3744:	00050593          	mv	a1,a0
    3748:	f6dff06f          	j	36b4 <.L23>

000000000000374c <.L51>:
    374c:	fe0606e3          	beqz	a2,3738 <.L91>
    3750:	00000513          	li	a0,0

0000000000003754 <.LVL219>:
    3754:	fff68713          	addi	a4,a3,-1
    3758:	fe9ff06f          	j	3740 <.L53>

000000000000375c <.L45>:
    375c:	00100693          	li	a3,1
    3760:	06071463          	bnez	a4,37c8 <.L54>

0000000000003764 <.LBB53>:
    3764:	0077f593          	andi	a1,a5,7
    3768:	00040693          	mv	a3,s0
    376c:	04058863          	beqz	a1,37bc <.L55>
    3770:	00200593          	li	a1,2
    3774:	00196913          	ori	s2,s2,1
    3778:	04b98063          	beq	s3,a1,37b8 <.L56>
    377c:	00300593          	li	a1,3
    3780:	02b98663          	beq	s3,a1,37ac <.L57>
    3784:	02099c63          	bnez	s3,37bc <.L55>
    3788:	00f7f593          	andi	a1,a5,15
    378c:	00400513          	li	a0,4
    3790:	02a58663          	beq	a1,a0,37bc <.L55>

0000000000003794 <.LBB54>:
    3794:	ffc7b693          	sltiu	a3,a5,-4

0000000000003798 <.L139>:
    3798:	0016c693          	xori	a3,a3,1
    379c:	02069693          	slli	a3,a3,0x20
    37a0:	0206d693          	srli	a3,a3,0x20
    37a4:	008686b3          	add	a3,a3,s0

00000000000037a8 <.LVL225>:
    37a8:	0140006f          	j	37bc <.L55>

00000000000037ac <.L57>:
    37ac:	00061863          	bnez	a2,37bc <.L55>

00000000000037b0 <.L140>:
    37b0:	ff87b693          	sltiu	a3,a5,-8
    37b4:	fe5ff06f          	j	3798 <.L139>

00000000000037b8 <.L56>:
    37b8:	fe061ce3          	bnez	a2,37b0 <.L140>

00000000000037bc <.L55>:
    37bc:	0346d693          	srli	a3,a3,0x34

00000000000037c0 <.LVL230>:
    37c0:	0016c693          	xori	a3,a3,1

00000000000037c4 <.LBE53>:
    37c4:	0016f693          	andi	a3,a3,1

00000000000037c8 <.L54>:
    37c8:	00100813          	li	a6,1
    37cc:	40e80833          	sub	a6,a6,a4

00000000000037d0 <.LVL232>:
    37d0:	07400713          	li	a4,116
    37d4:	11074263          	blt	a4,a6,38d8 <.L58>
    37d8:	03f00713          	li	a4,63
    37dc:	0008059b          	sext.w	a1,a6
    37e0:	07074463          	blt	a4,a6,3848 <.L59>
    37e4:	04000513          	li	a0,64
    37e8:	40b5053b          	subw	a0,a0,a1
    37ec:	00b7d833          	srl	a6,a5,a1

00000000000037f0 <.LVL233>:
    37f0:	00a41733          	sll	a4,s0,a0
    37f4:	00a79533          	sll	a0,a5,a0
    37f8:	01076733          	or	a4,a4,a6
    37fc:	00a03533          	snez	a0,a0
    3800:	00a767b3          	or	a5,a4,a0
    3804:	00b455b3          	srl	a1,s0,a1

0000000000003808 <.L60>:
    3808:	0077f713          	andi	a4,a5,7
    380c:	08070063          	beqz	a4,388c <.L62>
    3810:	00200713          	li	a4,2
    3814:	00196913          	ori	s2,s2,1
    3818:	06e98863          	beq	s3,a4,3888 <.L63>
    381c:	00300713          	li	a4,3
    3820:	04e98e63          	beq	s3,a4,387c <.L64>
    3824:	06099463          	bnez	s3,388c <.L62>
    3828:	00f7f713          	andi	a4,a5,15
    382c:	00400513          	li	a0,4
    3830:	04a70e63          	beq	a4,a0,388c <.L62>

0000000000003834 <.LBB59>:
    3834:	00478713          	addi	a4,a5,4

0000000000003838 <.L141>:
    3838:	00f737b3          	sltu	a5,a4,a5

000000000000383c <.LVL238>:
    383c:	00f585b3          	add	a1,a1,a5

0000000000003840 <.LVL239>:
    3840:	00070793          	mv	a5,a4
    3844:	0480006f          	j	388c <.L62>

0000000000003848 <.L59>:
    3848:	fc05871b          	addiw	a4,a1,-64
    384c:	04000893          	li	a7,64
    3850:	00e45733          	srl	a4,s0,a4
    3854:	00000513          	li	a0,0
    3858:	01180863          	beq	a6,a7,3868 <.L61>
    385c:	08000513          	li	a0,128
    3860:	40b5053b          	subw	a0,a0,a1
    3864:	00a41533          	sll	a0,s0,a0

0000000000003868 <.L61>:
    3868:	00f56533          	or	a0,a0,a5
    386c:	00a03533          	snez	a0,a0
    3870:	00a767b3          	or	a5,a4,a0

0000000000003874 <.LVL241>:
    3874:	00000593          	li	a1,0
    3878:	f91ff06f          	j	3808 <.L60>

000000000000387c <.L64>:
    387c:	00061863          	bnez	a2,388c <.L62>

0000000000003880 <.L142>:
    3880:	00878713          	addi	a4,a5,8
    3884:	fb5ff06f          	j	3838 <.L141>

0000000000003888 <.L63>:
    3888:	fe061ce3          	bnez	a2,3880 <.L142>

000000000000388c <.L62>:
    388c:	0335d713          	srli	a4,a1,0x33
    3890:	00177713          	andi	a4,a4,1
    3894:	02070063          	beqz	a4,38b4 <.L65>

0000000000003898 <.LVL243>:
    3898:	00196913          	ori	s2,s2,1
    389c:	00000593          	li	a1,0
    38a0:	00000513          	li	a0,0
    38a4:	00100713          	li	a4,1
    38a8:	e00686e3          	beqz	a3,36b4 <.L23>

00000000000038ac <.L66>:
    38ac:	00296913          	ori	s2,s2,2
    38b0:	e05ff06f          	j	36b4 <.L23>

00000000000038b4 <.L65>:
    38b4:	03d59513          	slli	a0,a1,0x3d
    38b8:	0037d793          	srli	a5,a5,0x3

00000000000038bc <.LVL248>:
    38bc:	00f56533          	or	a0,a0,a5

00000000000038c0 <.LVL249>:
    38c0:	0035d593          	srli	a1,a1,0x3
    38c4:	00000713          	li	a4,0
    38c8:	de0686e3          	beqz	a3,36b4 <.L23>
    38cc:	00197793          	andi	a5,s2,1
    38d0:	de0782e3          	beqz	a5,36b4 <.L23>
    38d4:	fd9ff06f          	j	38ac <.L66>

00000000000038d8 <.L58>:
    38d8:	0087e533          	or	a0,a5,s0
    38dc:	02050463          	beqz	a0,3904 <.L67>

00000000000038e0 <.LVL252>:
    38e0:	00200793          	li	a5,2
    38e4:	00196913          	ori	s2,s2,1
    38e8:	02f98e63          	beq	s3,a5,3924 <.L68>
    38ec:	00300793          	li	a5,3
    38f0:	02f98263          	beq	s3,a5,3914 <.L69>
    38f4:	00100513          	li	a0,1
    38f8:	00099463          	bnez	s3,3900 <.L70>
    38fc:	00500513          	li	a0,5

0000000000003900 <.L70>:
    3900:	00355513          	srli	a0,a0,0x3

0000000000003904 <.L67>:
    3904:	00296913          	ori	s2,s2,2
    3908:	00000593          	li	a1,0

000000000000390c <.L144>:
    390c:	00000713          	li	a4,0
    3910:	da5ff06f          	j	36b4 <.L23>

0000000000003914 <.L69>:
    3914:	00900513          	li	a0,9

0000000000003918 <.LBE64>:
    3918:	fe0604e3          	beqz	a2,3900 <.L70>

000000000000391c <.L143>:
    391c:	00100513          	li	a0,1
    3920:	fe1ff06f          	j	3900 <.L70>

0000000000003924 <.L68>:
    3924:	00900513          	li	a0,9

0000000000003928 <.LBE65>:
    3928:	fc061ce3          	bnez	a2,3900 <.L70>
    392c:	ff1ff06f          	j	391c <.L143>

0000000000003930 <.L21>:
    3930:	00100593          	li	a1,1
    3934:	00008737          	lui	a4,0x8
    3938:	02f59593          	slli	a1,a1,0x2f
    393c:	00000513          	li	a0,0
    3940:	fff70713          	addi	a4,a4,-1 # 7fff <.LLST3+0xd81>

0000000000003944 <.LVL260>:
    3944:	00000613          	li	a2,0
  FP_DIV_Q (R, A, B);
    3948:	01000913          	li	s2,16

000000000000394c <.LVL261>:
    394c:	d69ff06f          	j	36b4 <.L23>

0000000000003950 <.L88>:
  FP_PACK_Q (r, R);
    3950:	00100593          	li	a1,1
    3954:	00008737          	lui	a4,0x8
    3958:	02f59593          	slli	a1,a1,0x2f
    395c:	00000513          	li	a0,0
    3960:	fff70713          	addi	a4,a4,-1 # 7fff <.LLST3+0xd81>

0000000000003964 <.LVL263>:
    3964:	00000613          	li	a2,0
    3968:	d4dff06f          	j	36b4 <.L23>

000000000000396c <.L89>:
    396c:	00000593          	li	a1,0
    3970:	00000513          	li	a0,0
    3974:	f99ff06f          	j	390c <.L144>

0000000000003978 <__eqtf2>:
#include "soft-fp.h"
#include "quad.h"

CMPtype
__eqtf2 (TFtype a, TFtype b)
{
    3978:	00050813          	mv	a6,a0
  FP_DECL_EX;
  FP_DECL_Q (A);
  FP_DECL_Q (B);
  CMPtype r;

  FP_INIT_EXCEPTIONS;
    397c:	002027f3          	frrm	a5

0000000000003980 <.LBB2>:
  FP_UNPACK_RAW_Q (A, a);
    3980:	00008537          	lui	a0,0x8

0000000000003984 <.LVL2>:
    3984:	0305d893          	srli	a7,a1,0x30
    3988:	fff50513          	addi	a0,a0,-1 # 7fff <.LLST3+0xd81>
    398c:	fff00793          	li	a5,-1
    3990:	0107d793          	srli	a5,a5,0x10

0000000000003994 <.LBB3>:
  FP_UNPACK_RAW_Q (B, b);
    3994:	0306d313          	srli	t1,a3,0x30

0000000000003998 <.LBB4>:
  FP_UNPACK_RAW_Q (A, a);
    3998:	00a8f8b3          	and	a7,a7,a0
    399c:	00f5f733          	and	a4,a1,a5

00000000000039a0 <.LBB5>:
  FP_UNPACK_RAW_Q (B, b);
    39a0:	00a37333          	and	t1,t1,a0
    39a4:	00f6f7b3          	and	a5,a3,a5

00000000000039a8 <.LBB6>:
  FP_UNPACK_RAW_Q (A, a);
    39a8:	03f5d593          	srli	a1,a1,0x3f

00000000000039ac <.LBB7>:
  FP_UNPACK_RAW_Q (B, b);
    39ac:	03f6d693          	srli	a3,a3,0x3f

00000000000039b0 <.LBE7>:
  FP_CMP_EQ_Q (r, A, B, 1);
    39b0:	00a89a63          	bne	a7,a0,39c4 <.L2>
    39b4:	01076533          	or	a0,a4,a6
    39b8:	02051663          	bnez	a0,39e4 <.L3>
    39bc:	05131c63          	bne	t1,a7,3a14 <.L27>
    39c0:	0080006f          	j	39c8 <.L10>

00000000000039c4 <.L2>:
    39c4:	04a31c63          	bne	t1,a0,3a1c <.L5>

00000000000039c8 <.L10>:
    39c8:	00c7e533          	or	a0,a5,a2
    39cc:	04050863          	beqz	a0,3a1c <.L5>

00000000000039d0 <.LBB8>:
    39d0:	000086b7          	lui	a3,0x8

00000000000039d4 <.LVL7>:
    39d4:	fff68693          	addi	a3,a3,-1 # 7fff <.LLST3+0xd81>
    39d8:	02d89663          	bne	a7,a3,3a04 <.L6>
    39dc:	01076833          	or	a6,a4,a6

00000000000039e0 <.LVL8>:
    39e0:	02080263          	beqz	a6,3a04 <.L6>

00000000000039e4 <.L3>:
    39e4:	02f75713          	srli	a4,a4,0x2f

00000000000039e8 <.LVL10>:
    39e8:	06070063          	beqz	a4,3a48 <.L11>

00000000000039ec <.LBB9>:
    39ec:	00008737          	lui	a4,0x8
    39f0:	fff70713          	addi	a4,a4,-1 # 7fff <.LLST3+0xd81>

00000000000039f4 <.LBE9>:
    39f4:	00100513          	li	a0,1

00000000000039f8 <.LBB10>:
    39f8:	02e31063          	bne	t1,a4,3a18 <.L1>
    39fc:	00c7e633          	or	a2,a5,a2

0000000000003a00 <.LVL12>:
    3a00:	00060c63          	beqz	a2,3a18 <.L1>

0000000000003a04 <.L6>:
    3a04:	02f7d793          	srli	a5,a5,0x2f

0000000000003a08 <.LVL14>:
    3a08:	0017c793          	xori	a5,a5,1
    3a0c:	00479713          	slli	a4,a5,0x4

0000000000003a10 <.LBE10>:
  FP_HANDLE_EXCEPTIONS;
    3a10:	02079e63          	bnez	a5,3a4c <.L7>

0000000000003a14 <.L27>:
  FP_CMP_EQ_Q (r, A, B, 1);
    3a14:	00100513          	li	a0,1

0000000000003a18 <.L1>:

  return r;
}
    3a18:	00008067          	ret

0000000000003a1c <.L5>:
  FP_CMP_EQ_Q (r, A, B, 1);
    3a1c:	00100513          	li	a0,1
    3a20:	fe689ce3          	bne	a7,t1,3a18 <.L1>
    3a24:	fef71ae3          	bne	a4,a5,3a18 <.L1>
    3a28:	fec818e3          	bne	a6,a2,3a18 <.L1>
    3a2c:	00d58a63          	beq	a1,a3,3a40 <.L17>
    3a30:	fe0894e3          	bnez	a7,3a18 <.L1>
    3a34:	01076733          	or	a4,a4,a6

0000000000003a38 <.LVL18>:
    3a38:	00e03533          	snez	a0,a4
    3a3c:	00008067          	ret

0000000000003a40 <.L17>:
    3a40:	00000513          	li	a0,0

0000000000003a44 <.LVL20>:
  FP_HANDLE_EXCEPTIONS;
    3a44:	00008067          	ret

0000000000003a48 <.L11>:
  FP_CMP_EQ_Q (r, A, B, 1);
    3a48:	01000713          	li	a4,16

0000000000003a4c <.L7>:
  FP_HANDLE_EXCEPTIONS;
    3a4c:	00172073          	csrs	fflags,a4
    3a50:	fc5ff06f          	j	3a14 <.L27>

0000000000003a54 <__getf2>:
#include "soft-fp.h"
#include "quad.h"

CMPtype
__getf2 (TFtype a, TFtype b)
{
    3a54:	00050893          	mv	a7,a0
  FP_DECL_EX;
  FP_DECL_Q (A);
  FP_DECL_Q (B);
  CMPtype r;

  FP_INIT_EXCEPTIONS;
    3a58:	002027f3          	frrm	a5

0000000000003a5c <.LBB2>:
  FP_UNPACK_RAW_Q (A, a);
    3a5c:	00008337          	lui	t1,0x8
    3a60:	fff30313          	addi	t1,t1,-1 # 7fff <.LLST3+0xd81>
    3a64:	fff00793          	li	a5,-1
    3a68:	0305d813          	srli	a6,a1,0x30
    3a6c:	0107d793          	srli	a5,a5,0x10

0000000000003a70 <.LBB3>:
  FP_UNPACK_RAW_Q (B, b);
    3a70:	0306d713          	srli	a4,a3,0x30

0000000000003a74 <.LBB4>:
  FP_UNPACK_RAW_Q (A, a);
    3a74:	00687833          	and	a6,a6,t1
    3a78:	00f5fe33          	and	t3,a1,a5
    3a7c:	03f5d513          	srli	a0,a1,0x3f

0000000000003a80 <.LBB5>:
  FP_UNPACK_RAW_Q (B, b);
    3a80:	00f6f7b3          	and	a5,a3,a5
    3a84:	00677733          	and	a4,a4,t1
    3a88:	03f6d693          	srli	a3,a3,0x3f

0000000000003a8c <.LBE5>:
  FP_CMP_Q (r, A, B, -2, 2);
    3a8c:	00681c63          	bne	a6,t1,3aa4 <.L2>
    3a90:	011e65b3          	or	a1,t3,a7
    3a94:	08058a63          	beqz	a1,3b28 <.L31>

0000000000003a98 <.L3>:
  FP_HANDLE_EXCEPTIONS;
    3a98:	00186073          	csrsi	fflags,16
    3a9c:	ffe00513          	li	a0,-2

0000000000003aa0 <.LVL8>:
    3aa0:	00008067          	ret

0000000000003aa4 <.L2>:
  FP_CMP_Q (r, A, B, -2, 2);
    3aa4:	00671663          	bne	a4,t1,3ab0 <.L5>

0000000000003aa8 <.L20>:
    3aa8:	00c7e5b3          	or	a1,a5,a2
    3aac:	fe0596e3          	bnez	a1,3a98 <.L3>

0000000000003ab0 <.L5>:
    3ab0:	06081e63          	bnez	a6,3b2c <.L6>
    3ab4:	011e65b3          	or	a1,t3,a7
    3ab8:	0015b593          	seqz	a1,a1

0000000000003abc <.LVL10>:
    3abc:	04071663          	bnez	a4,3b08 <.L7>

0000000000003ac0 <.L19>:
    3ac0:	00c7e333          	or	t1,a5,a2
    3ac4:	04031263          	bnez	t1,3b08 <.L7>

0000000000003ac8 <.LVL12>:
    3ac8:	02059863          	bnez	a1,3af8 <.L35>

0000000000003acc <.L13>:
    3acc:	02050a63          	beqz	a0,3b00 <.L27>

0000000000003ad0 <.L29>:
    3ad0:	fff00513          	li	a0,-1

0000000000003ad4 <.LBE6>:

  return r;
    3ad4:	00008067          	ret

0000000000003ad8 <.L16>:
  FP_CMP_Q (r, A, B, -2, 2);
    3ad8:	fff00513          	li	a0,-1

0000000000003adc <.LVL16>:
    3adc:	04068063          	beqz	a3,3b1c <.L1>
    3ae0:	00068513          	mv	a0,a3
    3ae4:	00008067          	ret

0000000000003ae8 <.L12>:
    3ae8:	ffc7e2e3          	bltu	a5,t3,3acc <.L13>
    3aec:	02fe1a63          	bne	t3,a5,3b20 <.L14>
    3af0:	fd166ee3          	bltu	a2,a7,3acc <.L13>
    3af4:	02c8e263          	bltu	a7,a2,3b18 <.L18>

0000000000003af8 <.L35>:
    3af8:	00000513          	li	a0,0

0000000000003afc <.LVL19>:
    3afc:	00008067          	ret

0000000000003b00 <.L27>:
    3b00:	00100513          	li	a0,1

0000000000003b04 <.LVL21>:
    3b04:	00008067          	ret

0000000000003b08 <.L7>:
    3b08:	fc0598e3          	bnez	a1,3ad8 <.L16>

0000000000003b0c <.L17>:
    3b0c:	fcd510e3          	bne	a0,a3,3acc <.L13>
    3b10:	fb074ee3          	blt	a4,a6,3acc <.L13>
    3b14:	fce85ae3          	bge	a6,a4,3ae8 <.L12>

0000000000003b18 <.L18>:
    3b18:	fa050ce3          	beqz	a0,3ad0 <.L29>

0000000000003b1c <.L1>:
}
    3b1c:	00008067          	ret

0000000000003b20 <.L14>:
  FP_CMP_Q (r, A, B, -2, 2);
    3b20:	fefe6ce3          	bltu	t3,a5,3b18 <.L18>
    3b24:	fd5ff06f          	j	3af8 <.L35>

0000000000003b28 <.L31>:
    3b28:	f90700e3          	beq	a4,a6,3aa8 <.L20>

0000000000003b2c <.L6>:
    3b2c:	fe0710e3          	bnez	a4,3b0c <.L17>
    3b30:	00000593          	li	a1,0
    3b34:	f8dff06f          	j	3ac0 <.L19>

0000000000003b38 <__letf2>:
#include "soft-fp.h"
#include "quad.h"

CMPtype
__letf2 (TFtype a, TFtype b)
{
    3b38:	00050893          	mv	a7,a0
  FP_DECL_EX;
  FP_DECL_Q (A);
  FP_DECL_Q (B);
  CMPtype r;

  FP_INIT_EXCEPTIONS;
    3b3c:	002027f3          	frrm	a5

0000000000003b40 <.LBB2>:
  FP_UNPACK_RAW_Q (A, a);
    3b40:	00008337          	lui	t1,0x8
    3b44:	fff30313          	addi	t1,t1,-1 # 7fff <.LLST3+0xd81>
    3b48:	fff00793          	li	a5,-1
    3b4c:	0305d813          	srli	a6,a1,0x30
    3b50:	0107d793          	srli	a5,a5,0x10

0000000000003b54 <.LBB3>:
  FP_UNPACK_RAW_Q (B, b);
    3b54:	0306d713          	srli	a4,a3,0x30

0000000000003b58 <.LBB4>:
  FP_UNPACK_RAW_Q (A, a);
    3b58:	00687833          	and	a6,a6,t1
    3b5c:	00f5fe33          	and	t3,a1,a5
    3b60:	03f5d513          	srli	a0,a1,0x3f

0000000000003b64 <.LBB5>:
  FP_UNPACK_RAW_Q (B, b);
    3b64:	00f6f7b3          	and	a5,a3,a5
    3b68:	00677733          	and	a4,a4,t1
    3b6c:	03f6d693          	srli	a3,a3,0x3f

0000000000003b70 <.LBE5>:
  FP_CMP_Q (r, A, B, 2, 2);
    3b70:	00681c63          	bne	a6,t1,3b88 <.L2>
    3b74:	011e65b3          	or	a1,t3,a7
    3b78:	08058a63          	beqz	a1,3c0c <.L31>

0000000000003b7c <.L3>:
  FP_HANDLE_EXCEPTIONS;
    3b7c:	00186073          	csrsi	fflags,16
    3b80:	00200513          	li	a0,2

0000000000003b84 <.LVL8>:
    3b84:	00008067          	ret

0000000000003b88 <.L2>:
  FP_CMP_Q (r, A, B, 2, 2);
    3b88:	00671663          	bne	a4,t1,3b94 <.L5>

0000000000003b8c <.L20>:
    3b8c:	00c7e5b3          	or	a1,a5,a2
    3b90:	fe0596e3          	bnez	a1,3b7c <.L3>

0000000000003b94 <.L5>:
    3b94:	06081e63          	bnez	a6,3c10 <.L6>
    3b98:	011e65b3          	or	a1,t3,a7
    3b9c:	0015b593          	seqz	a1,a1

0000000000003ba0 <.LVL10>:
    3ba0:	04071663          	bnez	a4,3bec <.L7>

0000000000003ba4 <.L19>:
    3ba4:	00c7e333          	or	t1,a5,a2
    3ba8:	04031263          	bnez	t1,3bec <.L7>

0000000000003bac <.LVL12>:
    3bac:	02059863          	bnez	a1,3bdc <.L35>

0000000000003bb0 <.L13>:
    3bb0:	02050a63          	beqz	a0,3be4 <.L27>

0000000000003bb4 <.L29>:
    3bb4:	fff00513          	li	a0,-1

0000000000003bb8 <.LBE6>:

  return r;
    3bb8:	00008067          	ret

0000000000003bbc <.L16>:
  FP_CMP_Q (r, A, B, 2, 2);
    3bbc:	fff00513          	li	a0,-1

0000000000003bc0 <.LVL16>:
    3bc0:	04068063          	beqz	a3,3c00 <.L1>
    3bc4:	00068513          	mv	a0,a3
    3bc8:	00008067          	ret

0000000000003bcc <.L12>:
    3bcc:	ffc7e2e3          	bltu	a5,t3,3bb0 <.L13>
    3bd0:	02fe1a63          	bne	t3,a5,3c04 <.L14>
    3bd4:	fd166ee3          	bltu	a2,a7,3bb0 <.L13>
    3bd8:	02c8e263          	bltu	a7,a2,3bfc <.L18>

0000000000003bdc <.L35>:
    3bdc:	00000513          	li	a0,0

0000000000003be0 <.LVL19>:
    3be0:	00008067          	ret

0000000000003be4 <.L27>:
    3be4:	00100513          	li	a0,1

0000000000003be8 <.LVL21>:
    3be8:	00008067          	ret

0000000000003bec <.L7>:
    3bec:	fc0598e3          	bnez	a1,3bbc <.L16>

0000000000003bf0 <.L17>:
    3bf0:	fcd510e3          	bne	a0,a3,3bb0 <.L13>
    3bf4:	fb074ee3          	blt	a4,a6,3bb0 <.L13>
    3bf8:	fce85ae3          	bge	a6,a4,3bcc <.L12>

0000000000003bfc <.L18>:
    3bfc:	fa050ce3          	beqz	a0,3bb4 <.L29>

0000000000003c00 <.L1>:
}
    3c00:	00008067          	ret

0000000000003c04 <.L14>:
  FP_CMP_Q (r, A, B, 2, 2);
    3c04:	fefe6ce3          	bltu	t3,a5,3bfc <.L18>
    3c08:	fd5ff06f          	j	3bdc <.L35>

0000000000003c0c <.L31>:
    3c0c:	f90700e3          	beq	a4,a6,3b8c <.L20>

0000000000003c10 <.L6>:
    3c10:	fe0710e3          	bnez	a4,3bf0 <.L17>
    3c14:	00000593          	li	a1,0
    3c18:	f8dff06f          	j	3ba4 <.L19>

0000000000003c1c <__multf3>:
#include "soft-fp.h"
#include "quad.h"

TFtype
__multf3 (TFtype a, TFtype b)
{
    3c1c:	fa010113          	addi	sp,sp,-96
    3c20:	04813823          	sd	s0,80(sp)
    3c24:	03513423          	sd	s5,40(sp)
    3c28:	01813823          	sd	s8,16(sp)
    3c2c:	04113c23          	sd	ra,88(sp)
    3c30:	04913423          	sd	s1,72(sp)
    3c34:	05213023          	sd	s2,64(sp)
    3c38:	03313c23          	sd	s3,56(sp)
    3c3c:	03413823          	sd	s4,48(sp)
    3c40:	03613023          	sd	s6,32(sp)
    3c44:	01713c23          	sd	s7,24(sp)
    3c48:	01913423          	sd	s9,8(sp)
    3c4c:	00050413          	mv	s0,a0
    3c50:	00060c13          	mv	s8,a2
    3c54:	00068a93          	mv	s5,a3
  FP_DECL_Q (A);
  FP_DECL_Q (B);
  FP_DECL_Q (R);
  TFtype r;

  FP_INIT_ROUNDMODE;
    3c58:	00202973          	frrm	s2

0000000000003c5c <.LBB2>:
  FP_UNPACK_Q (A, a);
    3c5c:	00008737          	lui	a4,0x8
    3c60:	0305d793          	srli	a5,a1,0x30
    3c64:	fff70713          	addi	a4,a4,-1 # 7fff <.LLST3+0xd81>
    3c68:	01059993          	slli	s3,a1,0x10
    3c6c:	00e7f7b3          	and	a5,a5,a4

0000000000003c70 <.LBE2>:
  FP_INIT_ROUNDMODE;
    3c70:	0009091b          	sext.w	s2,s2

0000000000003c74 <.LBB3>:
  FP_UNPACK_Q (A, a);
    3c74:	0109d993          	srli	s3,s3,0x10

0000000000003c78 <.LVL2>:
    3c78:	03f5db13          	srli	s6,a1,0x3f

0000000000003c7c <.LBE3>:
    3c7c:	04078063          	beqz	a5,3cbc <.L2>
    3c80:	00078b9b          	sext.w	s7,a5
    3c84:	0ae78463          	beq	a5,a4,3d2c <.L3>

0000000000003c88 <.LVL4>:
    3c88:	00399993          	slli	s3,s3,0x3

0000000000003c8c <.LVL5>:
    3c8c:	03d55713          	srli	a4,a0,0x3d
    3c90:	01376733          	or	a4,a4,s3
    3c94:	ffffcbb7          	lui	s7,0xffffc
    3c98:	00100993          	li	s3,1
    3c9c:	03399993          	slli	s3,s3,0x33
    3ca0:	001b8b93          	addi	s7,s7,1 # ffffffffffffc001 <_ebss+0xffffffffffff33d1>
    3ca4:	013769b3          	or	s3,a4,s3

0000000000003ca8 <.LVL6>:
    3ca8:	00351a13          	slli	s4,a0,0x3

0000000000003cac <.LVL7>:
    3cac:	01778bb3          	add	s7,a5,s7

0000000000003cb0 <.L124>:
    3cb0:	00000c93          	li	s9,0

0000000000003cb4 <.L114>:
  FP_DECL_EX;
    3cb4:	00000493          	li	s1,0
    3cb8:	0940006f          	j	3d4c <.L4>

0000000000003cbc <.L2>:
  FP_UNPACK_Q (A, a);
    3cbc:	00a9ea33          	or	s4,s3,a0
    3cc0:	380a0463          	beqz	s4,4048 <.L57>

0000000000003cc4 <.LBB11>:
    3cc4:	04098663          	beqz	s3,3d10 <.L5>
    3cc8:	00098513          	mv	a0,s3

0000000000003ccc <.LVL11>:
    3ccc:	0bd010ef          	jal	ra,5588 <__clzdi2>

0000000000003cd0 <.LVL12>:
    3cd0:	0005079b          	sext.w	a5,a0

0000000000003cd4 <.L6>:
    3cd4:	ff178693          	addi	a3,a5,-15

0000000000003cd8 <.LVL14>:
    3cd8:	03c00613          	li	a2,60
    3cdc:	0006871b          	sext.w	a4,a3
    3ce0:	02d64e63          	blt	a2,a3,3d1c <.L7>
    3ce4:	03d00693          	li	a3,61

0000000000003ce8 <.LVL15>:
    3ce8:	00370a1b          	addiw	s4,a4,3
    3cec:	40e6873b          	subw	a4,a3,a4
    3cf0:	014999b3          	sll	s3,s3,s4

0000000000003cf4 <.LVL16>:
    3cf4:	00e45733          	srl	a4,s0,a4
    3cf8:	013769b3          	or	s3,a4,s3

0000000000003cfc <.LVL17>:
    3cfc:	01441a33          	sll	s4,s0,s4

0000000000003d00 <.L8>:
    3d00:	ffffcbb7          	lui	s7,0xffffc
    3d04:	011b8b93          	addi	s7,s7,17 # ffffffffffffc011 <_ebss+0xffffffffffff33e1>
    3d08:	40fb8bb3          	sub	s7,s7,a5
    3d0c:	fa5ff06f          	j	3cb0 <.L124>

0000000000003d10 <.L5>:
    3d10:	079010ef          	jal	ra,5588 <__clzdi2>

0000000000003d14 <.LVL20>:
    3d14:	0405079b          	addiw	a5,a0,64

0000000000003d18 <.LVL21>:
    3d18:	fbdff06f          	j	3cd4 <.L6>

0000000000003d1c <.L7>:
    3d1c:	fc37071b          	addiw	a4,a4,-61
    3d20:	00e419b3          	sll	s3,s0,a4

0000000000003d24 <.LVL23>:
    3d24:	00000a13          	li	s4,0
    3d28:	fd9ff06f          	j	3d00 <.L8>

0000000000003d2c <.L3>:
    3d2c:	00a9ea33          	or	s4,s3,a0
    3d30:	320a0463          	beqz	s4,4058 <.L58>

0000000000003d34 <.LVL25>:
    3d34:	02f9d493          	srli	s1,s3,0x2f
    3d38:	0014c493          	xori	s1,s1,1
    3d3c:	0014f493          	andi	s1,s1,1
    3d40:	00449493          	slli	s1,s1,0x4
    3d44:	00050a13          	mv	s4,a0
    3d48:	00300c93          	li	s9,3

0000000000003d4c <.L4>:
  FP_UNPACK_Q (B, b);
    3d4c:	000086b7          	lui	a3,0x8
    3d50:	030ad613          	srli	a2,s5,0x30
    3d54:	fff68693          	addi	a3,a3,-1 # 7fff <.LLST3+0xd81>
    3d58:	010a9413          	slli	s0,s5,0x10
    3d5c:	00d67633          	and	a2,a2,a3
    3d60:	000c0793          	mv	a5,s8

0000000000003d64 <.LVL27>:
    3d64:	01045413          	srli	s0,s0,0x10

0000000000003d68 <.LVL28>:
    3d68:	03fada93          	srli	s5,s5,0x3f

0000000000003d6c <.LBE12>:
    3d6c:	2e060c63          	beqz	a2,4064 <.L9>
    3d70:	0006071b          	sext.w	a4,a2
    3d74:	36d60263          	beq	a2,a3,40d8 <.L10>

0000000000003d78 <.LVL30>:
    3d78:	00341413          	slli	s0,s0,0x3

0000000000003d7c <.LVL31>:
    3d7c:	03dc5713          	srli	a4,s8,0x3d
    3d80:	00876733          	or	a4,a4,s0
    3d84:	00100413          	li	s0,1
    3d88:	03341413          	slli	s0,s0,0x33
    3d8c:	00876433          	or	s0,a4,s0

0000000000003d90 <.LVL32>:
    3d90:	ffffc737          	lui	a4,0xffffc
    3d94:	00170713          	addi	a4,a4,1 # ffffffffffffc001 <_ebss+0xffffffffffff33d1>
    3d98:	003c1793          	slli	a5,s8,0x3
    3d9c:	00e60733          	add	a4,a2,a4

0000000000003da0 <.L115>:
    3da0:	00000893          	li	a7,0

0000000000003da4 <.L11>:
  FP_MUL_Q (R, A, B);
    3da4:	017706b3          	add	a3,a4,s7
    3da8:	002c9713          	slli	a4,s9,0x2

0000000000003dac <.LVL36>:
    3dac:	01176733          	or	a4,a4,a7
    3db0:	00a00593          	li	a1,10
    3db4:	015b4633          	xor	a2,s6,s5

0000000000003db8 <.LVL37>:
    3db8:	00168813          	addi	a6,a3,1

0000000000003dbc <.LVL38>:
    3dbc:	3ae5c463          	blt	a1,a4,4164 <.L16>
    3dc0:	00200593          	li	a1,2
    3dc4:	34e5cc63          	blt	a1,a4,411c <.L17>
    3dc8:	fff70713          	addi	a4,a4,-1
    3dcc:	00100593          	li	a1,1
    3dd0:	36e5f863          	bgeu	a1,a4,4140 <.L19>

0000000000003dd4 <.L18>:
    3dd4:	fff00713          	li	a4,-1
    3dd8:	02075713          	srli	a4,a4,0x20
    3ddc:	014983b3          	add	t2,s3,s4

0000000000003de0 <.LVL40>:
    3de0:	00878f33          	add	t5,a5,s0

0000000000003de4 <.LBB21>:
    3de4:	020a5593          	srli	a1,s4,0x20
    3de8:	0207d513          	srli	a0,a5,0x20

0000000000003dec <.LBE21>:
    3dec:	0143bfb3          	sltu	t6,t2,s4
    3df0:	00ff32b3          	sltu	t0,t5,a5
    3df4:	00ea7a33          	and	s4,s4,a4

0000000000003df8 <.LVL41>:
    3df8:	00e7f7b3          	and	a5,a5,a4

0000000000003dfc <.LBB22>:
    3dfc:	034788b3          	mul	a7,a5,s4

0000000000003e00 <.LBE22>:
    3e00:	000f8e13          	mv	t3,t6

0000000000003e04 <.LVL44>:
    3e04:	00028313          	mv	t1,t0

0000000000003e08 <.LBB23>:
    3e08:	02a58733          	mul	a4,a1,a0
    3e0c:	02f587b3          	mul	a5,a1,a5

0000000000003e10 <.LVL46>:
    3e10:	03450533          	mul	a0,a0,s4

0000000000003e14 <.LVL47>:
    3e14:	00f505b3          	add	a1,a0,a5

0000000000003e18 <.LVL48>:
    3e18:	0208d513          	srli	a0,a7,0x20

0000000000003e1c <.LVL49>:
    3e1c:	00b50533          	add	a0,a0,a1

0000000000003e20 <.LVL50>:
    3e20:	00f57863          	bgeu	a0,a5,3e30 <.L24>
    3e24:	00100793          	li	a5,1

0000000000003e28 <.LVL51>:
    3e28:	02079793          	slli	a5,a5,0x20
    3e2c:	00f70733          	add	a4,a4,a5

0000000000003e30 <.L24>:
    3e30:	fff00793          	li	a5,-1
    3e34:	0207d793          	srli	a5,a5,0x20
    3e38:	02055e93          	srli	t4,a0,0x20
    3e3c:	00f57533          	and	a0,a0,a5

0000000000003e40 <.LVL53>:
    3e40:	00f8f8b3          	and	a7,a7,a5

0000000000003e44 <.LVL54>:
    3e44:	02051513          	slli	a0,a0,0x20

0000000000003e48 <.LBB24>:
    3e48:	0203da93          	srli	s5,t2,0x20

0000000000003e4c <.LBB25>:
    3e4c:	00ee8eb3          	add	t4,t4,a4

0000000000003e50 <.LVL56>:
    3e50:	01150533          	add	a0,a0,a7

0000000000003e54 <.LBB26>:
    3e54:	020f5713          	srli	a4,t5,0x20

0000000000003e58 <.LVL58>:
    3e58:	00f3f8b3          	and	a7,t2,a5
    3e5c:	00ff77b3          	and	a5,t5,a5
    3e60:	031785b3          	mul	a1,a5,a7

0000000000003e64 <.LVL59>:
    3e64:	02fa87b3          	mul	a5,s5,a5

0000000000003e68 <.LVL60>:
    3e68:	02ea8ab3          	mul	s5,s5,a4

0000000000003e6c <.LVL61>:
    3e6c:	03170733          	mul	a4,a4,a7
    3e70:	00f708b3          	add	a7,a4,a5

0000000000003e74 <.LVL62>:
    3e74:	0205d713          	srli	a4,a1,0x20

0000000000003e78 <.LVL63>:
    3e78:	01170733          	add	a4,a4,a7

0000000000003e7c <.LVL64>:
    3e7c:	00f77863          	bgeu	a4,a5,3e8c <.L25>
    3e80:	00100793          	li	a5,1

0000000000003e84 <.LVL65>:
    3e84:	02079793          	slli	a5,a5,0x20
    3e88:	00fa8ab3          	add	s5,s5,a5

0000000000003e8c <.L25>:
    3e8c:	fff00793          	li	a5,-1

0000000000003e90 <.LVL67>:
    3e90:	0207d793          	srli	a5,a5,0x20

0000000000003e94 <.LVL68>:
    3e94:	02075a13          	srli	s4,a4,0x20
    3e98:	00f77733          	and	a4,a4,a5

0000000000003e9c <.LVL69>:
    3e9c:	00f5f5b3          	and	a1,a1,a5

0000000000003ea0 <.LVL70>:
    3ea0:	02071713          	slli	a4,a4,0x20

0000000000003ea4 <.LBB27>:
    3ea4:	0209d893          	srli	a7,s3,0x20

0000000000003ea8 <.LBB28>:
    3ea8:	00b70733          	add	a4,a4,a1

0000000000003eac <.LBB29>:
    3eac:	00f9f9b3          	and	s3,s3,a5

0000000000003eb0 <.LVL73>:
    3eb0:	02045593          	srli	a1,s0,0x20

0000000000003eb4 <.LVL74>:
    3eb4:	00f477b3          	and	a5,s0,a5
    3eb8:	02f98b33          	mul	s6,s3,a5

0000000000003ebc <.LVL75>:
    3ebc:	02b88433          	mul	s0,a7,a1

0000000000003ec0 <.LVL76>:
    3ec0:	02f887b3          	mul	a5,a7,a5

0000000000003ec4 <.LVL77>:
    3ec4:	020b5893          	srli	a7,s6,0x20

0000000000003ec8 <.LVL78>:
    3ec8:	033585b3          	mul	a1,a1,s3

0000000000003ecc <.LVL79>:
    3ecc:	00f585b3          	add	a1,a1,a5

0000000000003ed0 <.LVL80>:
    3ed0:	00b888b3          	add	a7,a7,a1

0000000000003ed4 <.LVL81>:
    3ed4:	00f8f863          	bgeu	a7,a5,3ee4 <.L26>
    3ed8:	00100793          	li	a5,1

0000000000003edc <.LVL82>:
    3edc:	02079793          	slli	a5,a5,0x20
    3ee0:	00f40433          	add	s0,s0,a5

0000000000003ee4 <.L26>:
    3ee4:	0208d593          	srli	a1,a7,0x20
    3ee8:	008585b3          	add	a1,a1,s0

0000000000003eec <.LVL84>:
    3eec:	fff00413          	li	s0,-1

0000000000003ef0 <.LVL85>:
    3ef0:	02045413          	srli	s0,s0,0x20

0000000000003ef4 <.LBB30>:
    3ef4:	01d70733          	add	a4,a4,t4

0000000000003ef8 <.LBB31>:
    3ef8:	0088f7b3          	and	a5,a7,s0

0000000000003efc <.LBB32>:
    3efc:	01d738b3          	sltu	a7,a4,t4

0000000000003f00 <.LBB33>:
    3f00:	008b7b33          	and	s6,s6,s0

0000000000003f04 <.LBE33>:
    3f04:	40600333          	neg	t1,t1

0000000000003f08 <.LBB34>:
    3f08:	011a0433          	add	s0,s4,a7
    3f0c:	01540433          	add	s0,s0,s5

0000000000003f10 <.LBE34>:
    3f10:	00737333          	and	t1,t1,t2
    3f14:	41c00e33          	neg	t3,t3

0000000000003f18 <.LBB35>:
    3f18:	00830333          	add	t1,t1,s0

0000000000003f1c <.LBB36>:
    3f1c:	005fffb3          	and	t6,t6,t0

0000000000003f20 <.LBE36>:
    3f20:	01ee7e33          	and	t3,t3,t5

0000000000003f24 <.LBB37>:
    3f24:	011438b3          	sltu	a7,s0,a7

0000000000003f28 <.LBB38>:
    3f28:	006e0e33          	add	t3,t3,t1

0000000000003f2c <.LBB39>:
    3f2c:	01f888b3          	add	a7,a7,t6

0000000000003f30 <.LBB40>:
    3f30:	00833433          	sltu	s0,t1,s0
    3f34:	008888b3          	add	a7,a7,s0

0000000000003f38 <.LBB41>:
    3f38:	006e3333          	sltu	t1,t3,t1

0000000000003f3c <.LBB42>:
    3f3c:	40a70f33          	sub	t5,a4,a0

0000000000003f40 <.LVL91>:
    3f40:	01e73fb3          	sltu	t6,a4,t5

0000000000003f44 <.LBB47>:
    3f44:	02079793          	slli	a5,a5,0x20

0000000000003f48 <.LBB48>:
    3f48:	006888b3          	add	a7,a7,t1

0000000000003f4c <.LBB49>:
    3f4c:	41de0333          	sub	t1,t3,t4
    3f50:	006e32b3          	sltu	t0,t3,t1

0000000000003f54 <.LBB50>:
    3f54:	016787b3          	add	a5,a5,s6

0000000000003f58 <.LBB51>:
    3f58:	41f30333          	sub	t1,t1,t6

0000000000003f5c <.LVL94>:
    3f5c:	00000f93          	li	t6,0
    3f60:	01e77663          	bgeu	a4,t5,3f6c <.L27>
    3f64:	41ce8eb3          	sub	t4,t4,t3

0000000000003f68 <.LVL95>:
    3f68:	001ebf93          	seqz	t6,t4

0000000000003f6c <.L27>:
    3f6c:	40ff0e33          	sub	t3,t5,a5

0000000000003f70 <.LVL96>:
    3f70:	00b888b3          	add	a7,a7,a1
    3f74:	01cf3733          	sltu	a4,t5,t3

0000000000003f78 <.LVL97>:
    3f78:	40b305b3          	sub	a1,t1,a1

0000000000003f7c <.LBB59>:
    3f7c:	005fefb3          	or	t6,t6,t0

0000000000003f80 <.LBB60>:
    3f80:	00b33333          	sltu	t1,t1,a1

0000000000003f84 <.LVL100>:
    3f84:	40e78733          	sub	a4,a5,a4

0000000000003f88 <.LVL101>:
    3f88:	00000e93          	li	t4,0
    3f8c:	01cf7463          	bgeu	t5,t3,3f94 <.L28>
    3f90:	0015be93          	seqz	t4,a1

0000000000003f94 <.L28>:
    3f94:	00e585b3          	add	a1,a1,a4

0000000000003f98 <.LBB62>:
    3f98:	00f5b733          	sltu	a4,a1,a5
    3f9c:	01170733          	add	a4,a4,a7
    3fa0:	41f70733          	sub	a4,a4,t6

0000000000003fa4 <.LBB63>:
    3fa4:	006eeeb3          	or	t4,t4,t1

0000000000003fa8 <.LBB64>:
    3fa8:	41d70733          	sub	a4,a4,t4

0000000000003fac <.LBB67>:
    3fac:	00de1793          	slli	a5,t3,0xd

0000000000003fb0 <.LVL105>:
    3fb0:	00d71713          	slli	a4,a4,0xd
    3fb4:	0335d413          	srli	s0,a1,0x33
    3fb8:	00a7e7b3          	or	a5,a5,a0

0000000000003fbc <.LVL106>:
    3fbc:	00876433          	or	s0,a4,s0

0000000000003fc0 <.LBB68>:
    3fc0:	00f037b3          	snez	a5,a5

0000000000003fc4 <.LVL108>:
    3fc4:	033e5513          	srli	a0,t3,0x33

0000000000003fc8 <.LBE19>:
    3fc8:	03475713          	srli	a4,a4,0x34

0000000000003fcc <.LBB72>:
    3fcc:	00a7e7b3          	or	a5,a5,a0

0000000000003fd0 <.LBB69>:
    3fd0:	00d59593          	slli	a1,a1,0xd

0000000000003fd4 <.LBE69>:
    3fd4:	00177713          	andi	a4,a4,1

0000000000003fd8 <.LBB73>:
    3fd8:	00b7e7b3          	or	a5,a5,a1

0000000000003fdc <.LBE73>:
    3fdc:	1c070263          	beqz	a4,41a0 <.L67>
    3fe0:	0017d713          	srli	a4,a5,0x1
    3fe4:	0017f793          	andi	a5,a5,1

0000000000003fe8 <.LVL112>:
    3fe8:	00f76733          	or	a4,a4,a5
    3fec:	03f41793          	slli	a5,s0,0x3f
    3ff0:	00f767b3          	or	a5,a4,a5

0000000000003ff4 <.LVL113>:
    3ff4:	00145413          	srli	s0,s0,0x1

0000000000003ff8 <.L29>:
  FP_PACK_Q (r, R);
    3ff8:	00004737          	lui	a4,0x4
    3ffc:	fff70713          	addi	a4,a4,-1 # 3fff <.L29+0x7>
    4000:	00e80733          	add	a4,a6,a4

0000000000004004 <.LVL115>:
    4004:	28e05e63          	blez	a4,42a0 <.L30>
    4008:	0077f693          	andi	a3,a5,7
    400c:	1a068663          	beqz	a3,41b8 <.L31>
    4010:	00200693          	li	a3,2
    4014:	0014e493          	ori	s1,s1,1
    4018:	18d90e63          	beq	s2,a3,41b4 <.L32>
    401c:	00300693          	li	a3,3
    4020:	18d90463          	beq	s2,a3,41a8 <.L33>
    4024:	18091a63          	bnez	s2,41b8 <.L31>
    4028:	00f7f693          	andi	a3,a5,15
    402c:	00400593          	li	a1,4
    4030:	18b68463          	beq	a3,a1,41b8 <.L31>

0000000000004034 <.LBB77>:
    4034:	00478693          	addi	a3,a5,4

0000000000004038 <.L116>:
    4038:	00f6b7b3          	sltu	a5,a3,a5

000000000000403c <.LVL118>:
    403c:	00f40433          	add	s0,s0,a5

0000000000004040 <.LVL119>:
    4040:	00068793          	mv	a5,a3
    4044:	1740006f          	j	41b8 <.L31>

0000000000004048 <.L57>:
  FP_UNPACK_Q (A, a);
    4048:	00000993          	li	s3,0

000000000000404c <.LBE79>:
    404c:	00000b93          	li	s7,0
    4050:	00100c93          	li	s9,1
    4054:	c61ff06f          	j	3cb4 <.L114>

0000000000004058 <.L58>:
    4058:	00000993          	li	s3,0

000000000000405c <.LBE80>:
    405c:	00200c93          	li	s9,2
    4060:	c55ff06f          	j	3cb4 <.L114>

0000000000004064 <.L9>:
  FP_UNPACK_Q (B, b);
    4064:	008c67b3          	or	a5,s8,s0

0000000000004068 <.LVL125>:
    4068:	08078863          	beqz	a5,40f8 <.L59>

000000000000406c <.LBB81>:
    406c:	04040663          	beqz	s0,40b8 <.L12>
    4070:	00040513          	mv	a0,s0
    4074:	514010ef          	jal	ra,5588 <__clzdi2>

0000000000004078 <.LVL126>:
    4078:	0005051b          	sext.w	a0,a0

000000000000407c <.L13>:
    407c:	ff150793          	addi	a5,a0,-15

0000000000004080 <.LVL128>:
    4080:	03c00693          	li	a3,60
    4084:	0007871b          	sext.w	a4,a5
    4088:	04f6c063          	blt	a3,a5,40c8 <.L14>
    408c:	03d00693          	li	a3,61
    4090:	0037079b          	addiw	a5,a4,3

0000000000004094 <.LVL129>:
    4094:	40e6873b          	subw	a4,a3,a4
    4098:	00f41433          	sll	s0,s0,a5

000000000000409c <.LVL130>:
    409c:	00ec5733          	srl	a4,s8,a4
    40a0:	00876433          	or	s0,a4,s0

00000000000040a4 <.LVL131>:
    40a4:	00fc17b3          	sll	a5,s8,a5

00000000000040a8 <.L15>:
    40a8:	ffffc737          	lui	a4,0xffffc
    40ac:	01170713          	addi	a4,a4,17 # ffffffffffffc011 <_ebss+0xffffffffffff33e1>
    40b0:	40a70733          	sub	a4,a4,a0
    40b4:	cedff06f          	j	3da0 <.L115>

00000000000040b8 <.L12>:
    40b8:	000c0513          	mv	a0,s8
    40bc:	4cc010ef          	jal	ra,5588 <__clzdi2>

00000000000040c0 <.LVL134>:
    40c0:	0405051b          	addiw	a0,a0,64

00000000000040c4 <.LVL135>:
    40c4:	fb9ff06f          	j	407c <.L13>

00000000000040c8 <.L14>:
    40c8:	fc37071b          	addiw	a4,a4,-61
    40cc:	00ec1433          	sll	s0,s8,a4

00000000000040d0 <.LVL137>:
    40d0:	00000793          	li	a5,0

00000000000040d4 <.LVL138>:
    40d4:	fd5ff06f          	j	40a8 <.L15>

00000000000040d8 <.L10>:
    40d8:	008c66b3          	or	a3,s8,s0
    40dc:	02068863          	beqz	a3,410c <.L60>

00000000000040e0 <.LVL140>:
    40e0:	02f45693          	srli	a3,s0,0x2f
    40e4:	0016f693          	andi	a3,a3,1
    40e8:	00300893          	li	a7,3
    40ec:	ca069ce3          	bnez	a3,3da4 <.L11>
    40f0:	01000493          	li	s1,16

00000000000040f4 <.LVL141>:
    40f4:	cb1ff06f          	j	3da4 <.L11>

00000000000040f8 <.L59>:
    40f8:	00000413          	li	s0,0

00000000000040fc <.LVL143>:
    40fc:	00000793          	li	a5,0

0000000000004100 <.LBE82>:
    4100:	00000713          	li	a4,0
    4104:	00100893          	li	a7,1
    4108:	c9dff06f          	j	3da4 <.L11>

000000000000410c <.L60>:
    410c:	00000413          	li	s0,0

0000000000004110 <.LVL145>:
    4110:	00000793          	li	a5,0

0000000000004114 <.LBE83>:
    4114:	00200893          	li	a7,2
    4118:	c8dff06f          	j	3da4 <.L11>

000000000000411c <.L17>:
    411c:	00100593          	li	a1,1
    4120:	00e59733          	sll	a4,a1,a4
  FP_MUL_Q (R, A, B);
    4124:	53077513          	andi	a0,a4,1328
    4128:	04051863          	bnez	a0,4178 <.L62>
    412c:	24077313          	andi	t1,a4,576
    4130:	34031263          	bnez	t1,4474 <.L63>
    4134:	08877713          	andi	a4,a4,136
    4138:	c8070ee3          	beqz	a4,3dd4 <.L18>

000000000000413c <.L21>:
  FP_UNPACK_Q (B, b);
    413c:	000a8613          	mv	a2,s5

0000000000004140 <.L19>:
  FP_PACK_Q (r, R);
    4140:	00200713          	li	a4,2
    4144:	34e88463          	beq	a7,a4,448c <.L68>

0000000000004148 <.L22>:
    4148:	00300713          	li	a4,3
    414c:	34e88a63          	beq	a7,a4,44a0 <.L69>
    4150:	00100713          	li	a4,1
    4154:	eae892e3          	bne	a7,a4,3ff8 <.L29>
    4158:	00000593          	li	a1,0
    415c:	00000513          	li	a0,0
    4160:	2f00006f          	j	4450 <.L123>

0000000000004164 <.L16>:
  FP_MUL_Q (R, A, B);
    4164:	00f00693          	li	a3,15
    4168:	02d70063          	beq	a4,a3,4188 <.L64>
    416c:	00b00693          	li	a3,11
    4170:	fcd706e3          	beq	a4,a3,413c <.L21>

0000000000004174 <.LBB85>:
  FP_UNPACK_Q (A, a);
    4174:	000b0613          	mv	a2,s6

0000000000004178 <.L62>:
  FP_MUL_Q (R, A, B);
    4178:	00098413          	mv	s0,s3

000000000000417c <.LVL154>:
    417c:	000a0793          	mv	a5,s4

0000000000004180 <.LVL155>:
    4180:	000c8893          	mv	a7,s9

0000000000004184 <.LVL156>:
    4184:	fbdff06f          	j	4140 <.L19>

0000000000004188 <.L64>:
    4188:	00100413          	li	s0,1

000000000000418c <.LVL158>:
    418c:	02f41413          	slli	s0,s0,0x2f
    4190:	00000793          	li	a5,0

0000000000004194 <.LVL159>:
    4194:	00000613          	li	a2,0
    4198:	00300893          	li	a7,3

000000000000419c <.LVL160>:
    419c:	fadff06f          	j	4148 <.L22>

00000000000041a0 <.L67>:
    41a0:	00068813          	mv	a6,a3

00000000000041a4 <.LVL162>:
    41a4:	e55ff06f          	j	3ff8 <.L29>

00000000000041a8 <.L33>:
  FP_PACK_Q (r, R);
    41a8:	00061863          	bnez	a2,41b8 <.L31>

00000000000041ac <.L117>:
    41ac:	00878693          	addi	a3,a5,8
    41b0:	e89ff06f          	j	4038 <.L116>

00000000000041b4 <.L32>:
    41b4:	fe061ce3          	bnez	a2,41ac <.L117>

00000000000041b8 <.L31>:
    41b8:	03445693          	srli	a3,s0,0x34
    41bc:	0016f693          	andi	a3,a3,1
    41c0:	00068e63          	beqz	a3,41dc <.L34>
    41c4:	fff00713          	li	a4,-1

00000000000041c8 <.LVL164>:
    41c8:	03471713          	slli	a4,a4,0x34
    41cc:	fff70713          	addi	a4,a4,-1
    41d0:	00e47433          	and	s0,s0,a4
    41d4:	00004737          	lui	a4,0x4

00000000000041d8 <.LVL166>:
    41d8:	00e80733          	add	a4,a6,a4

00000000000041dc <.L34>:
    41dc:	000086b7          	lui	a3,0x8
    41e0:	ffe68593          	addi	a1,a3,-2 # 7ffe <.LLST3+0xd80>
    41e4:	06e5c863          	blt	a1,a4,4254 <.L35>
    41e8:	0037d793          	srli	a5,a5,0x3

00000000000041ec <.LVL168>:
    41ec:	03d41513          	slli	a0,s0,0x3d

00000000000041f0 <.LVL169>:
    41f0:	00f56533          	or	a0,a0,a5

00000000000041f4 <.LVL170>:
    41f4:	00345593          	srli	a1,s0,0x3

00000000000041f8 <.L20>:
    41f8:	03171713          	slli	a4,a4,0x31
    41fc:	03175713          	srli	a4,a4,0x31
    4200:	00f61613          	slli	a2,a2,0xf
    4204:	01059593          	slli	a1,a1,0x10
    4208:	00e66633          	or	a2,a2,a4
    420c:	03061613          	slli	a2,a2,0x30
    4210:	0105d593          	srli	a1,a1,0x10
    4214:	00c5e5b3          	or	a1,a1,a2

0000000000004218 <.LBE89>:
  FP_HANDLE_EXCEPTIONS;
    4218:	00048463          	beqz	s1,4220 <.L1>
    421c:	0014a073          	csrs	fflags,s1

0000000000004220 <.L1>:

  return r;
}
    4220:	05813083          	ld	ra,88(sp)
    4224:	05013403          	ld	s0,80(sp)
    4228:	04813483          	ld	s1,72(sp)

000000000000422c <.LVL173>:
    422c:	04013903          	ld	s2,64(sp)

0000000000004230 <.LVL174>:
    4230:	03813983          	ld	s3,56(sp)
    4234:	03013a03          	ld	s4,48(sp)
    4238:	02813a83          	ld	s5,40(sp)
    423c:	02013b03          	ld	s6,32(sp)
    4240:	01813b83          	ld	s7,24(sp)

0000000000004244 <.LVL175>:
    4244:	01013c03          	ld	s8,16(sp)
    4248:	00813c83          	ld	s9,8(sp)

000000000000424c <.LVL176>:
    424c:	06010113          	addi	sp,sp,96
    4250:	00008067          	ret

0000000000004254 <.L35>:
  FP_PACK_Q (r, R);
    4254:	00200793          	li	a5,2

0000000000004258 <.LVL178>:
    4258:	02f90c63          	beq	s2,a5,4290 <.L36>
    425c:	00300793          	li	a5,3
    4260:	fff68693          	addi	a3,a3,-1
    4264:	00f90a63          	beq	s2,a5,4278 <.L37>
    4268:	00091a63          	bnez	s2,427c <.L71>

000000000000426c <.L72>:
    426c:	00000513          	li	a0,0

0000000000004270 <.LVL179>:
    4270:	00068713          	mv	a4,a3
    4274:	0100006f          	j	4284 <.L38>

0000000000004278 <.L37>:
    4278:	fe060ae3          	beqz	a2,426c <.L72>

000000000000427c <.L71>:
    427c:	fff00513          	li	a0,-1

0000000000004280 <.LVL181>:
    4280:	00058713          	mv	a4,a1

0000000000004284 <.L38>:
    4284:	0054e493          	ori	s1,s1,5

0000000000004288 <.LVL183>:
    4288:	00050593          	mv	a1,a0
    428c:	f6dff06f          	j	41f8 <.L20>

0000000000004290 <.L36>:
    4290:	fe0606e3          	beqz	a2,427c <.L71>
    4294:	00000513          	li	a0,0

0000000000004298 <.LVL185>:
    4298:	fff68713          	addi	a4,a3,-1
    429c:	fe9ff06f          	j	4284 <.L38>

00000000000042a0 <.L30>:
    42a0:	00100693          	li	a3,1
    42a4:	06071463          	bnez	a4,430c <.L39>

00000000000042a8 <.LBB91>:
    42a8:	0077f593          	andi	a1,a5,7
    42ac:	00040693          	mv	a3,s0
    42b0:	04058863          	beqz	a1,4300 <.L40>
    42b4:	00200593          	li	a1,2
    42b8:	0014e493          	ori	s1,s1,1
    42bc:	04b90063          	beq	s2,a1,42fc <.L41>
    42c0:	00300593          	li	a1,3
    42c4:	02b90663          	beq	s2,a1,42f0 <.L42>
    42c8:	02091c63          	bnez	s2,4300 <.L40>
    42cc:	00f7f593          	andi	a1,a5,15
    42d0:	00400513          	li	a0,4
    42d4:	02a58663          	beq	a1,a0,4300 <.L40>

00000000000042d8 <.LBB92>:
    42d8:	ffc7b693          	sltiu	a3,a5,-4

00000000000042dc <.L118>:
    42dc:	0016c693          	xori	a3,a3,1
    42e0:	02069693          	slli	a3,a3,0x20
    42e4:	0206d693          	srli	a3,a3,0x20
    42e8:	008686b3          	add	a3,a3,s0

00000000000042ec <.LVL191>:
    42ec:	0140006f          	j	4300 <.L40>

00000000000042f0 <.L42>:
    42f0:	00061863          	bnez	a2,4300 <.L40>

00000000000042f4 <.L119>:
    42f4:	ff87b693          	sltiu	a3,a5,-8
    42f8:	fe5ff06f          	j	42dc <.L118>

00000000000042fc <.L41>:
    42fc:	fe061ce3          	bnez	a2,42f4 <.L119>

0000000000004300 <.L40>:
    4300:	0346d693          	srli	a3,a3,0x34

0000000000004304 <.LVL196>:
    4304:	0016c693          	xori	a3,a3,1

0000000000004308 <.LBE91>:
    4308:	0016f693          	andi	a3,a3,1

000000000000430c <.L39>:
    430c:	00100813          	li	a6,1
    4310:	40e80833          	sub	a6,a6,a4

0000000000004314 <.LVL198>:
    4314:	07400713          	li	a4,116
    4318:	11074263          	blt	a4,a6,441c <.L43>
    431c:	03f00713          	li	a4,63
    4320:	0008059b          	sext.w	a1,a6
    4324:	07074463          	blt	a4,a6,438c <.L44>
    4328:	04000713          	li	a4,64
    432c:	40b7073b          	subw	a4,a4,a1
    4330:	00e41533          	sll	a0,s0,a4
    4334:	00b7d833          	srl	a6,a5,a1

0000000000004338 <.LVL199>:
    4338:	00e79733          	sll	a4,a5,a4
    433c:	01056533          	or	a0,a0,a6
    4340:	00e03733          	snez	a4,a4
    4344:	00e56533          	or	a0,a0,a4

0000000000004348 <.LVL200>:
    4348:	00b455b3          	srl	a1,s0,a1

000000000000434c <.L45>:
    434c:	00757793          	andi	a5,a0,7
    4350:	08078063          	beqz	a5,43d0 <.L47>
    4354:	00200793          	li	a5,2
    4358:	0014e493          	ori	s1,s1,1
    435c:	06f90863          	beq	s2,a5,43cc <.L48>
    4360:	00300793          	li	a5,3
    4364:	04f90e63          	beq	s2,a5,43c0 <.L49>
    4368:	06091463          	bnez	s2,43d0 <.L47>
    436c:	00f57793          	andi	a5,a0,15
    4370:	00400713          	li	a4,4
    4374:	04e78e63          	beq	a5,a4,43d0 <.L47>

0000000000004378 <.LBB97>:
    4378:	00450793          	addi	a5,a0,4

000000000000437c <.L120>:
    437c:	00a7b533          	sltu	a0,a5,a0

0000000000004380 <.LVL204>:
    4380:	00a585b3          	add	a1,a1,a0

0000000000004384 <.LVL205>:
    4384:	00078513          	mv	a0,a5
    4388:	0480006f          	j	43d0 <.L47>

000000000000438c <.L44>:
    438c:	fc05851b          	addiw	a0,a1,-64
    4390:	04000893          	li	a7,64
    4394:	00a45533          	srl	a0,s0,a0
    4398:	00000713          	li	a4,0
    439c:	01180863          	beq	a6,a7,43ac <.L46>
    43a0:	08000713          	li	a4,128
    43a4:	40b7073b          	subw	a4,a4,a1
    43a8:	00e41733          	sll	a4,s0,a4

00000000000043ac <.L46>:
    43ac:	00f76733          	or	a4,a4,a5
    43b0:	00e03733          	snez	a4,a4
    43b4:	00e56533          	or	a0,a0,a4

00000000000043b8 <.LVL207>:
    43b8:	00000593          	li	a1,0
    43bc:	f91ff06f          	j	434c <.L45>

00000000000043c0 <.L49>:
    43c0:	00061863          	bnez	a2,43d0 <.L47>

00000000000043c4 <.L121>:
    43c4:	00850793          	addi	a5,a0,8
    43c8:	fb5ff06f          	j	437c <.L120>

00000000000043cc <.L48>:
    43cc:	fe061ce3          	bnez	a2,43c4 <.L121>

00000000000043d0 <.L47>:
    43d0:	0335d793          	srli	a5,a1,0x33
    43d4:	0017f793          	andi	a5,a5,1
    43d8:	02078063          	beqz	a5,43f8 <.L50>

00000000000043dc <.LVL209>:
    43dc:	0014e493          	ori	s1,s1,1
    43e0:	00000593          	li	a1,0
    43e4:	00000513          	li	a0,0
    43e8:	00100713          	li	a4,1
    43ec:	e00686e3          	beqz	a3,41f8 <.L20>

00000000000043f0 <.L51>:
    43f0:	0024e493          	ori	s1,s1,2
    43f4:	e05ff06f          	j	41f8 <.L20>

00000000000043f8 <.L50>:
    43f8:	03d59793          	slli	a5,a1,0x3d
    43fc:	00355513          	srli	a0,a0,0x3

0000000000004400 <.LVL214>:
    4400:	00a7e533          	or	a0,a5,a0

0000000000004404 <.LVL215>:
    4404:	0035d593          	srli	a1,a1,0x3
    4408:	00000713          	li	a4,0
    440c:	de0686e3          	beqz	a3,41f8 <.L20>
    4410:	0014f793          	andi	a5,s1,1
    4414:	de0782e3          	beqz	a5,41f8 <.L20>
    4418:	fd9ff06f          	j	43f0 <.L51>

000000000000441c <.L43>:
    441c:	0087e533          	or	a0,a5,s0
    4420:	02050463          	beqz	a0,4448 <.L52>

0000000000004424 <.LVL218>:
    4424:	00200793          	li	a5,2
    4428:	0014e493          	ori	s1,s1,1
    442c:	02f90e63          	beq	s2,a5,4468 <.L53>
    4430:	00300793          	li	a5,3
    4434:	02f90263          	beq	s2,a5,4458 <.L54>
    4438:	00100513          	li	a0,1
    443c:	00091463          	bnez	s2,4444 <.L55>
    4440:	00500513          	li	a0,5

0000000000004444 <.L55>:
    4444:	00355513          	srli	a0,a0,0x3

0000000000004448 <.L52>:
    4448:	0024e493          	ori	s1,s1,2
    444c:	00000593          	li	a1,0

0000000000004450 <.L123>:
    4450:	00000713          	li	a4,0
    4454:	da5ff06f          	j	41f8 <.L20>

0000000000004458 <.L54>:
    4458:	00900513          	li	a0,9

000000000000445c <.LBE102>:
    445c:	fe0604e3          	beqz	a2,4444 <.L55>

0000000000004460 <.L122>:
    4460:	00100513          	li	a0,1
    4464:	fe1ff06f          	j	4444 <.L55>

0000000000004468 <.L53>:
    4468:	00900513          	li	a0,9

000000000000446c <.LBE103>:
    446c:	fc061ce3          	bnez	a2,4444 <.L55>
    4470:	ff1ff06f          	j	4460 <.L122>

0000000000004474 <.L63>:
  FP_MUL_Q (R, A, B);
    4474:	00008737          	lui	a4,0x8
    4478:	02f59593          	slli	a1,a1,0x2f
    447c:	fff70713          	addi	a4,a4,-1 # 7fff <.LLST3+0xd81>

0000000000004480 <.LVL226>:
    4480:	00000613          	li	a2,0
    4484:	01000493          	li	s1,16

0000000000004488 <.LVL227>:
    4488:	d71ff06f          	j	41f8 <.L20>

000000000000448c <.L68>:
  FP_PACK_Q (r, R);
    448c:	00008737          	lui	a4,0x8
    4490:	00000593          	li	a1,0
    4494:	00000513          	li	a0,0
    4498:	fff70713          	addi	a4,a4,-1 # 7fff <.LLST3+0xd81>
    449c:	d5dff06f          	j	41f8 <.L20>

00000000000044a0 <.L69>:
    44a0:	00100593          	li	a1,1
    44a4:	00008737          	lui	a4,0x8
    44a8:	02f59593          	slli	a1,a1,0x2f
    44ac:	00000513          	li	a0,0
    44b0:	fff70713          	addi	a4,a4,-1 # 7fff <.LLST3+0xd81>

00000000000044b4 <.LVL229>:
    44b4:	00000613          	li	a2,0
    44b8:	d41ff06f          	j	41f8 <.L20>

00000000000044bc <__subtf3>:
#include "soft-fp.h"
#include "quad.h"

TFtype
__subtf3 (TFtype a, TFtype b)
{
    44bc:	fd010113          	addi	sp,sp,-48
    44c0:	02113423          	sd	ra,40(sp)
    44c4:	02813023          	sd	s0,32(sp)
    44c8:	00913c23          	sd	s1,24(sp)
    44cc:	01213823          	sd	s2,16(sp)
    44d0:	01313423          	sd	s3,8(sp)
    44d4:	01413023          	sd	s4,0(sp)
  FP_DECL_Q (A);
  FP_DECL_Q (B);
  FP_DECL_Q (R);
  TFtype r;

  FP_INIT_ROUNDMODE;
    44d8:	002024f3          	frrm	s1

00000000000044dc <.LBB2>:
  FP_UNPACK_SEMIRAW_Q (A, a);
    44dc:	fff00713          	li	a4,-1
    44e0:	01075713          	srli	a4,a4,0x10
    44e4:	00008e37          	lui	t3,0x8
    44e8:	0305d813          	srli	a6,a1,0x30
    44ec:	fffe0e13          	addi	t3,t3,-1 # 7fff <.LLST3+0xd81>

00000000000044f0 <.LBB3>:
  FP_UNPACK_SEMIRAW_Q (B, b);
    44f0:	0306d893          	srli	a7,a3,0x30

00000000000044f4 <.LBB4>:
  FP_UNPACK_SEMIRAW_Q (A, a);
    44f4:	03f5d913          	srli	s2,a1,0x3f

00000000000044f8 <.LBB5>:
  FP_UNPACK_SEMIRAW_Q (B, b);
    44f8:	03f6d313          	srli	t1,a3,0x3f

00000000000044fc <.LBB6>:
  FP_UNPACK_SEMIRAW_Q (A, a);
    44fc:	00e5f5b3          	and	a1,a1,a4

0000000000004500 <.LBB7>:
  FP_UNPACK_SEMIRAW_Q (B, b);
    4500:	00e6f6b3          	and	a3,a3,a4

0000000000004504 <.LBE7>:
  FP_UNPACK_SEMIRAW_Q (A, a);
    4504:	03d55793          	srli	a5,a0,0x3d
  FP_UNPACK_SEMIRAW_Q (B, b);
    4508:	00369713          	slli	a4,a3,0x3

000000000000450c <.LBB10>:
  FP_UNPACK_SEMIRAW_Q (A, a);
    450c:	01c87833          	and	a6,a6,t3

0000000000004510 <.LBE10>:
  FP_UNPACK_SEMIRAW_Q (B, b);
    4510:	03d65693          	srli	a3,a2,0x3d
  FP_UNPACK_SEMIRAW_Q (A, a);
    4514:	00359593          	slli	a1,a1,0x3

0000000000004518 <.LBB13>:
  FP_UNPACK_SEMIRAW_Q (B, b);
    4518:	01c8f8b3          	and	a7,a7,t3

000000000000451c <.LBE13>:
  FP_INIT_ROUNDMODE;
    451c:	0004849b          	sext.w	s1,s1

0000000000004520 <.LBB14>:
  FP_UNPACK_SEMIRAW_Q (A, a);
    4520:	00080413          	mv	s0,a6

0000000000004524 <.LBE14>:
    4524:	00b7e7b3          	or	a5,a5,a1

0000000000004528 <.LVL3>:
    4528:	00351513          	slli	a0,a0,0x3

000000000000452c <.LBB16>:
  FP_UNPACK_SEMIRAW_Q (B, b);
    452c:	00e6e6b3          	or	a3,a3,a4

0000000000004530 <.LVL5>:
    4530:	00361613          	slli	a2,a2,0x3

0000000000004534 <.LVL6>:
  FP_SUB_Q (R, A, B);
    4534:	01c89663          	bne	a7,t3,4540 <.L2>
    4538:	00c6e733          	or	a4,a3,a2
    453c:	00071463          	bnez	a4,4544 <.L3>

0000000000004540 <.L2>:
    4540:	00134313          	xori	t1,t1,1

0000000000004544 <.L3>:
    4544:	4118073b          	subw	a4,a6,a7
    4548:	00008e37          	lui	t3,0x8
    454c:	0007059b          	sext.w	a1,a4
    4550:	fffe0e93          	addi	t4,t3,-1 # 7fff <.LLST3+0xd81>

0000000000004554 <.LBE18>:
    4554:	4f231c63          	bne	t1,s2,4a4c <.L4>

0000000000004558 <.LBB39>:
    4558:	14b05a63          	blez	a1,46ac <.L5>
    455c:	0a089a63          	bnez	a7,4610 <.L6>
    4560:	00c6e5b3          	or	a1,a3,a2
    4564:	02059263          	bnez	a1,4588 <.L7>

0000000000004568 <.L224>:
    4568:	15d81ce3          	bne	a6,t4,4ec0 <.L79>
    456c:	00a7e733          	or	a4,a5,a0
    4570:	1c070ee3          	beqz	a4,4f4c <.L120>
    4574:	0327d713          	srli	a4,a5,0x32
    4578:	00174713          	xori	a4,a4,1
    457c:	00177713          	andi	a4,a4,1
    4580:	00471713          	slli	a4,a4,0x4
    4584:	1600006f          	j	46e4 <.L9>

0000000000004588 <.L7>:
    4588:	fff7059b          	addiw	a1,a4,-1

000000000000458c <.LVL10>:
    458c:	06059263          	bnez	a1,45f0 <.L10>

0000000000004590 <.LBB19>:
    4590:	00c50633          	add	a2,a0,a2

0000000000004594 <.LVL11>:
    4594:	00d786b3          	add	a3,a5,a3

0000000000004598 <.LVL12>:
    4598:	00a637b3          	sltu	a5,a2,a0

000000000000459c <.LVL13>:
    459c:	00f687b3          	add	a5,a3,a5

00000000000045a0 <.LBB20>:
    45a0:	00060513          	mv	a0,a2

00000000000045a4 <.L11>:
    45a4:	0337d713          	srli	a4,a5,0x33
    45a8:	00177713          	andi	a4,a4,1
    45ac:	10070ae3          	beqz	a4,4ec0 <.L79>

00000000000045b0 <.LVL16>:
    45b0:	00008737          	lui	a4,0x8
    45b4:	00140413          	addi	s0,s0,1

00000000000045b8 <.LVL17>:
    45b8:	fff70693          	addi	a3,a4,-1 # 7fff <.LLST3+0xd81>
    45bc:	44d40e63          	beq	s0,a3,4a18 <.L38>
    45c0:	fff00713          	li	a4,-1
    45c4:	03371713          	slli	a4,a4,0x33
    45c8:	fff70713          	addi	a4,a4,-1
    45cc:	00e7f7b3          	and	a5,a5,a4

00000000000045d0 <.LVL18>:
    45d0:	00155713          	srli	a4,a0,0x1
    45d4:	00157513          	andi	a0,a0,1
    45d8:	00a76733          	or	a4,a4,a0
    45dc:	03f79513          	slli	a0,a5,0x3f
    45e0:	00e56533          	or	a0,a0,a4
    45e4:	0017d793          	srli	a5,a5,0x1

00000000000045e8 <.L133>:
  FP_PACK_SEMIRAW_Q (r, R);
    45e8:	00000593          	li	a1,0
    45ec:	3ac0006f          	j	4998 <.L203>

00000000000045f0 <.L10>:
  FP_SUB_Q (R, A, B);
    45f0:	03d81863          	bne	a6,t4,4620 <.L12>

00000000000045f4 <.L218>:
    45f4:	00a7e733          	or	a4,a5,a0
    45f8:	14070ae3          	beqz	a4,4f4c <.L120>
    45fc:	0327d713          	srli	a4,a5,0x32
    4600:	00174713          	xori	a4,a4,1
    4604:	00177713          	andi	a4,a4,1
    4608:	00471713          	slli	a4,a4,0x4
    460c:	1400006f          	j	474c <.L202>

0000000000004610 <.L6>:
    4610:	ffd802e3          	beq	a6,t4,45f4 <.L218>
    4614:	00100713          	li	a4,1
    4618:	03371713          	slli	a4,a4,0x33
    461c:	00e6e6b3          	or	a3,a3,a4

0000000000004620 <.L12>:
    4620:	07400713          	li	a4,116
    4624:	06b74e63          	blt	a4,a1,46a0 <.L14>
    4628:	03f00713          	li	a4,63
    462c:	04b74063          	blt	a4,a1,466c <.L15>
    4630:	04000813          	li	a6,64
    4634:	40b8083b          	subw	a6,a6,a1
    4638:	01069733          	sll	a4,a3,a6
    463c:	00b658b3          	srl	a7,a2,a1

0000000000004640 <.LVL23>:
    4640:	01061833          	sll	a6,a2,a6
    4644:	01176733          	or	a4,a4,a7
    4648:	01003833          	snez	a6,a6
    464c:	01076733          	or	a4,a4,a6

0000000000004650 <.LVL24>:
    4650:	00b6d5b3          	srl	a1,a3,a1

0000000000004654 <.L16>:
    4654:	00a70733          	add	a4,a4,a0

0000000000004658 <.LVL26>:
    4658:	00f585b3          	add	a1,a1,a5

000000000000465c <.LVL27>:
    465c:	00a737b3          	sltu	a5,a4,a0

0000000000004660 <.LVL28>:
    4660:	00f587b3          	add	a5,a1,a5

0000000000004664 <.LBB25>:
    4664:	00070513          	mv	a0,a4

0000000000004668 <.LBE25>:
    4668:	f3dff06f          	j	45a4 <.L11>

000000000000466c <.L15>:
    466c:	fc05871b          	addiw	a4,a1,-64
    4670:	04000893          	li	a7,64

0000000000004674 <.LVL32>:
    4674:	00e6d733          	srl	a4,a3,a4
    4678:	00000813          	li	a6,0
    467c:	01158863          	beq	a1,a7,468c <.L17>
    4680:	08000813          	li	a6,128
    4684:	40b8083b          	subw	a6,a6,a1
    4688:	01069833          	sll	a6,a3,a6

000000000000468c <.L17>:
    468c:	00c86833          	or	a6,a6,a2
    4690:	01003833          	snez	a6,a6
    4694:	01076733          	or	a4,a4,a6

0000000000004698 <.L190>:
    4698:	00000593          	li	a1,0

000000000000469c <.LVL34>:
    469c:	fb9ff06f          	j	4654 <.L16>

00000000000046a0 <.L14>:
    46a0:	00c6e733          	or	a4,a3,a2
    46a4:	00e03733          	snez	a4,a4
    46a8:	ff1ff06f          	j	4698 <.L190>

00000000000046ac <.L5>:
    46ac:	14058263          	beqz	a1,47f0 <.L18>

00000000000046b0 <.LVL37>:
    46b0:	0a081263          	bnez	a6,4754 <.L19>
    46b4:	00a7e5b3          	or	a1,a5,a0
    46b8:	04059463          	bnez	a1,4700 <.L20>
    46bc:	01d896e3          	bne	a7,t4,4ec8 <.L101>
    46c0:	00c6e533          	or	a0,a3,a2

00000000000046c4 <.LVL38>:
    46c4:	06050ce3          	beqz	a0,4f3c <.L104>
    46c8:	0326d713          	srli	a4,a3,0x32

00000000000046cc <.LVL39>:
    46cc:	00174713          	xori	a4,a4,1
    46d0:	00177713          	andi	a4,a4,1
    46d4:	00471713          	slli	a4,a4,0x4

00000000000046d8 <.LBE88>:
  FP_UNPACK_SEMIRAW_Q (B, b);
    46d8:	00068793          	mv	a5,a3

00000000000046dc <.LVL40>:
    46dc:	00060513          	mv	a0,a2

00000000000046e0 <.LBB90>:
    46e0:	00088413          	mv	s0,a7

00000000000046e4 <.L9>:
  FP_PACK_SEMIRAW_Q (r, R);
    46e4:	00757693          	andi	a3,a0,7
    46e8:	2e069a63          	bnez	a3,49dc <.L188>

00000000000046ec <.LVL42>:
    46ec:	1c058e63          	beqz	a1,48c8 <.L87>
    46f0:	00177693          	andi	a3,a4,1
    46f4:	1c068a63          	beqz	a3,48c8 <.L87>

00000000000046f8 <.L94>:
    46f8:	00276713          	ori	a4,a4,2
    46fc:	1cc0006f          	j	48c8 <.L87>

0000000000004700 <.L20>:
  FP_SUB_Q (R, A, B);
    4700:	fff74713          	not	a4,a4

0000000000004704 <.LVL45>:
    4704:	0007071b          	sext.w	a4,a4

0000000000004708 <.LVL46>:
    4708:	00071e63          	bnez	a4,4724 <.L21>

000000000000470c <.LBB27>:
    470c:	00c50533          	add	a0,a0,a2

0000000000004710 <.LVL47>:
    4710:	00d786b3          	add	a3,a5,a3

0000000000004714 <.LVL48>:
    4714:	00c53633          	sltu	a2,a0,a2

0000000000004718 <.LVL49>:
    4718:	00c687b3          	add	a5,a3,a2

000000000000471c <.L192>:
  FP_UNPACK_SEMIRAW_Q (B, b);
    471c:	00088413          	mv	s0,a7

0000000000004720 <.LBB94>:
    4720:	e85ff06f          	j	45a4 <.L11>

0000000000004724 <.L21>:
  FP_SUB_Q (R, A, B);
    4724:	05d89263          	bne	a7,t4,4768 <.L22>

0000000000004728 <.L214>:
    4728:	00c6e533          	or	a0,a3,a2

000000000000472c <.LVL53>:
    472c:	000508e3          	beqz	a0,4f3c <.L104>
    4730:	0326d713          	srli	a4,a3,0x32
    4734:	00174713          	xori	a4,a4,1
    4738:	00177713          	andi	a4,a4,1
    473c:	00471713          	slli	a4,a4,0x4

0000000000004740 <.LBE94>:
  FP_UNPACK_SEMIRAW_Q (B, b);
    4740:	00068793          	mv	a5,a3

0000000000004744 <.LVL54>:
    4744:	00060513          	mv	a0,a2

0000000000004748 <.LBB96>:
  FP_SUB_Q (R, A, B);
    4748:	00088413          	mv	s0,a7

000000000000474c <.L202>:
  FP_PACK_SEMIRAW_Q (r, R);
    474c:	00000593          	li	a1,0

0000000000004750 <.LBB98>:
    4750:	f95ff06f          	j	46e4 <.L9>

0000000000004754 <.L19>:
  FP_SUB_Q (R, A, B);
    4754:	fdd88ae3          	beq	a7,t4,4728 <.L214>
    4758:	00100593          	li	a1,1
    475c:	03359593          	slli	a1,a1,0x33
    4760:	40e0073b          	negw	a4,a4

0000000000004764 <.LVL57>:
    4764:	00b7e7b3          	or	a5,a5,a1

0000000000004768 <.L22>:
    4768:	07400593          	li	a1,116
    476c:	06e5cc63          	blt	a1,a4,47e4 <.L24>
    4770:	03f00593          	li	a1,63
    4774:	02e5ce63          	blt	a1,a4,47b0 <.L25>
    4778:	04000813          	li	a6,64
    477c:	40e8083b          	subw	a6,a6,a4
    4780:	010795b3          	sll	a1,a5,a6
    4784:	00e55333          	srl	t1,a0,a4

0000000000004788 <.LVL59>:
    4788:	01051833          	sll	a6,a0,a6
    478c:	0065e5b3          	or	a1,a1,t1
    4790:	01003833          	snez	a6,a6
    4794:	0105e5b3          	or	a1,a1,a6

0000000000004798 <.LVL60>:
    4798:	00e7d733          	srl	a4,a5,a4

000000000000479c <.L26>:
    479c:	00c58533          	add	a0,a1,a2

00000000000047a0 <.LVL62>:
    47a0:	00d70733          	add	a4,a4,a3

00000000000047a4 <.LVL63>:
    47a4:	00c53633          	sltu	a2,a0,a2

00000000000047a8 <.LVL64>:
    47a8:	00c707b3          	add	a5,a4,a2
    47ac:	f71ff06f          	j	471c <.L192>

00000000000047b0 <.L25>:
    47b0:	fc07059b          	addiw	a1,a4,-64
    47b4:	04000313          	li	t1,64

00000000000047b8 <.LVL66>:
    47b8:	00b7d5b3          	srl	a1,a5,a1
    47bc:	00000813          	li	a6,0
    47c0:	00670863          	beq	a4,t1,47d0 <.L27>
    47c4:	08000813          	li	a6,128
    47c8:	40e8083b          	subw	a6,a6,a4
    47cc:	01079833          	sll	a6,a5,a6

00000000000047d0 <.L27>:
    47d0:	00a86833          	or	a6,a6,a0
    47d4:	01003833          	snez	a6,a6
    47d8:	0105e5b3          	or	a1,a1,a6

00000000000047dc <.L191>:
    47dc:	00000713          	li	a4,0

00000000000047e0 <.LVL68>:
    47e0:	fbdff06f          	j	479c <.L26>

00000000000047e4 <.L24>:
    47e4:	00a7e5b3          	or	a1,a5,a0
    47e8:	00b035b3          	snez	a1,a1
    47ec:	ff1ff06f          	j	47dc <.L191>

00000000000047f0 <.L18>:
    47f0:	00180413          	addi	s0,a6,1
    47f4:	ffee0713          	addi	a4,t3,-2
    47f8:	00e47333          	and	t1,s0,a4

00000000000047fc <.LVL71>:
    47fc:	16031c63          	bnez	t1,4974 <.L28>
    4800:	00a7e333          	or	t1,a5,a0
    4804:	04081663          	bnez	a6,4850 <.L29>

0000000000004808 <.LVL72>:
    4808:	6e030263          	beqz	t1,4eec <.L106>

000000000000480c <.LBE99>:
    480c:	00c6e733          	or	a4,a3,a2

0000000000004810 <.LBB100>:
    4810:	48070863          	beqz	a4,4ca0 <.L30>

0000000000004814 <.LBB32>:
    4814:	00c50633          	add	a2,a0,a2

0000000000004818 <.LVL73>:
    4818:	00d786b3          	add	a3,a5,a3

000000000000481c <.LVL74>:
    481c:	00a637b3          	sltu	a5,a2,a0

0000000000004820 <.LVL75>:
    4820:	00f687b3          	add	a5,a3,a5

0000000000004824 <.LBE32>:
    4824:	0337d713          	srli	a4,a5,0x33
    4828:	00177713          	andi	a4,a4,1
    482c:	6c070263          	beqz	a4,4ef0 <.L107>
    4830:	fff00713          	li	a4,-1
    4834:	03371713          	slli	a4,a4,0x33
    4838:	fff70713          	addi	a4,a4,-1
    483c:	00e7f7b3          	and	a5,a5,a4

0000000000004840 <.LBB33>:
    4840:	00060513          	mv	a0,a2

0000000000004844 <.LBE33>:
    4844:	00000713          	li	a4,0
    4848:	00100413          	li	s0,1
    484c:	e99ff06f          	j	46e4 <.L9>

0000000000004850 <.L29>:
    4850:	03d81c63          	bne	a6,t4,4888 <.L31>
    4854:	78030e63          	beqz	t1,4ff0 <.L32>
    4858:	0327d713          	srli	a4,a5,0x32
    485c:	00174713          	xori	a4,a4,1
    4860:	00177713          	andi	a4,a4,1
    4864:	00471713          	slli	a4,a4,0x4

0000000000004868 <.LVL80>:
    4868:	05089063          	bne	a7,a6,48a8 <.L34>

000000000000486c <.L33>:
    486c:	00c6e833          	or	a6,a3,a2
    4870:	02080063          	beqz	a6,4890 <.L35>
    4874:	0326d813          	srli	a6,a3,0x32
    4878:	00187813          	andi	a6,a6,1
    487c:	00081a63          	bnez	a6,4890 <.L35>
    4880:	01000713          	li	a4,16
    4884:	00c0006f          	j	4890 <.L35>

0000000000004888 <.L31>:
    4888:	00000713          	li	a4,0
    488c:	ffd880e3          	beq	a7,t4,486c <.L33>

0000000000004890 <.L35>:
    4890:	00031c63          	bnez	t1,48a8 <.L34>

0000000000004894 <.L96>:
  FP_UNPACK_SEMIRAW_Q (B, b);
    4894:	00068793          	mv	a5,a3

0000000000004898 <.LVL85>:
    4898:	00060513          	mv	a0,a2

000000000000489c <.L131>:
  FP_SUB_Q (R, A, B);
    489c:	00008437          	lui	s0,0x8
    48a0:	fff40413          	addi	s0,s0,-1 # 7fff <.LLST3+0xd81>
    48a4:	e41ff06f          	j	46e4 <.L9>

00000000000048a8 <.L34>:
    48a8:	00c6e6b3          	or	a3,a3,a2

00000000000048ac <.LVL88>:
    48ac:	fe0688e3          	beqz	a3,489c <.L131>

00000000000048b0 <.L205>:
    48b0:	00100793          	li	a5,1

00000000000048b4 <.LVL90>:
    48b4:	00008437          	lui	s0,0x8
    48b8:	00000913          	li	s2,0

00000000000048bc <.LVL91>:
    48bc:	03279793          	slli	a5,a5,0x32
    48c0:	00000513          	li	a0,0

00000000000048c4 <.LVL92>:
    48c4:	fff40413          	addi	s0,s0,-1 # 7fff <.LLST3+0xd81>

00000000000048c8 <.L87>:
  FP_PACK_SEMIRAW_Q (r, R);
    48c8:	0337d693          	srli	a3,a5,0x33
    48cc:	0016f693          	andi	a3,a3,1
    48d0:	02068263          	beqz	a3,48f4 <.L88>

00000000000048d4 <.LVL94>:
    48d4:	000086b7          	lui	a3,0x8
    48d8:	00140413          	addi	s0,s0,1

00000000000048dc <.LVL95>:
    48dc:	fff68613          	addi	a2,a3,-1 # 7fff <.LLST3+0xd81>
    48e0:	6cc40663          	beq	s0,a2,4fac <.L89>
    48e4:	fff00693          	li	a3,-1
    48e8:	03369693          	slli	a3,a3,0x33
    48ec:	fff68693          	addi	a3,a3,-1
    48f0:	00d7f7b3          	and	a5,a5,a3

00000000000048f4 <.L88>:
    48f4:	00355693          	srli	a3,a0,0x3
    48f8:	03d79513          	slli	a0,a5,0x3d

00000000000048fc <.LVL97>:
    48fc:	00d56533          	or	a0,a0,a3

0000000000004900 <.LVL98>:
    4900:	000086b7          	lui	a3,0x8
    4904:	fff68693          	addi	a3,a3,-1 # 7fff <.LLST3+0xd81>
    4908:	0037d793          	srli	a5,a5,0x3
    490c:	02d41063          	bne	s0,a3,492c <.L92>

0000000000004910 <.LVL100>:
    4910:	00f56533          	or	a0,a0,a5

0000000000004914 <.LVL101>:
    4914:	00000793          	li	a5,0

0000000000004918 <.LVL102>:
    4918:	00050a63          	beqz	a0,492c <.L92>
    491c:	00100793          	li	a5,1
    4920:	02f79793          	slli	a5,a5,0x2f

0000000000004924 <.LVL103>:
    4924:	00000513          	li	a0,0
    4928:	00000913          	li	s2,0

000000000000492c <.L92>:
    492c:	03141413          	slli	s0,s0,0x31
    4930:	00f9191b          	slliw	s2,s2,0xf

0000000000004934 <.LVL105>:
    4934:	03145413          	srli	s0,s0,0x31
    4938:	01079793          	slli	a5,a5,0x10

000000000000493c <.LVL106>:
    493c:	01246433          	or	s0,s0,s2
    4940:	03041413          	slli	s0,s0,0x30
    4944:	0107d593          	srli	a1,a5,0x10
    4948:	0085e5b3          	or	a1,a1,s0

000000000000494c <.LBE106>:
  FP_HANDLE_EXCEPTIONS;
    494c:	00070463          	beqz	a4,4954 <.L1>
    4950:	00172073          	csrs	fflags,a4

0000000000004954 <.L1>:

  return r;
}
    4954:	02813083          	ld	ra,40(sp)
    4958:	02013403          	ld	s0,32(sp)
    495c:	01813483          	ld	s1,24(sp)

0000000000004960 <.LVL108>:
    4960:	01013903          	ld	s2,16(sp)
    4964:	00813983          	ld	s3,8(sp)
    4968:	00013a03          	ld	s4,0(sp)
    496c:	03010113          	addi	sp,sp,48
    4970:	00008067          	ret

0000000000004974 <.L28>:
  FP_SUB_Q (R, A, B);
    4974:	03d40663          	beq	s0,t4,49a0 <.L36>

0000000000004978 <.LBB36>:
    4978:	00c50633          	add	a2,a0,a2

000000000000497c <.LVL110>:
    497c:	00a63533          	sltu	a0,a2,a0

0000000000004980 <.LVL111>:
    4980:	00d786b3          	add	a3,a5,a3

0000000000004984 <.LVL112>:
    4984:	00a686b3          	add	a3,a3,a0

0000000000004988 <.LBE36>:
    4988:	03f69513          	slli	a0,a3,0x3f
    498c:	00165613          	srli	a2,a2,0x1

0000000000004990 <.LVL113>:
    4990:	00c56533          	or	a0,a0,a2
    4994:	0016d793          	srli	a5,a3,0x1

0000000000004998 <.L203>:
    4998:	00000713          	li	a4,0
    499c:	d49ff06f          	j	46e4 <.L9>

00000000000049a0 <.L36>:
    49a0:	00048863          	beqz	s1,49b0 <.L117>
    49a4:	00300793          	li	a5,3

00000000000049a8 <.LVL116>:
    49a8:	00f49c63          	bne	s1,a5,49c0 <.L37>
    49ac:	02091063          	bnez	s2,49cc <.L112>

00000000000049b0 <.L117>:
    49b0:	00000793          	li	a5,0
    49b4:	00000513          	li	a0,0
    49b8:	00500713          	li	a4,5
    49bc:	f0dff06f          	j	48c8 <.L87>

00000000000049c0 <.L37>:
    49c0:	00200793          	li	a5,2
    49c4:	00f49463          	bne	s1,a5,49cc <.L112>
    49c8:	fe0914e3          	bnez	s2,49b0 <.L117>

00000000000049cc <.L112>:
    49cc:	fff00793          	li	a5,-1
    49d0:	fff00513          	li	a0,-1

00000000000049d4 <.LVL119>:
    49d4:	00070413          	mv	s0,a4

00000000000049d8 <.L204>:
    49d8:	00500713          	li	a4,5

00000000000049dc <.L188>:
  FP_PACK_SEMIRAW_Q (r, R);
    49dc:	00200693          	li	a3,2
    49e0:	00176713          	ori	a4,a4,1
    49e4:	5cd48063          	beq	s1,a3,4fa4 <.L84>
    49e8:	00300693          	li	a3,3
    49ec:	5ad48663          	beq	s1,a3,4f98 <.L85>
    49f0:	02049063          	bnez	s1,4a10 <.L86>
    49f4:	00f57693          	andi	a3,a0,15
    49f8:	00400613          	li	a2,4
    49fc:	00c68a63          	beq	a3,a2,4a10 <.L86>

0000000000004a00 <.LBB76>:
    4a00:	00450693          	addi	a3,a0,4

0000000000004a04 <.L208>:
    4a04:	00a6b533          	sltu	a0,a3,a0
    4a08:	00a787b3          	add	a5,a5,a0

0000000000004a0c <.LVL124>:
    4a0c:	00068513          	mv	a0,a3

0000000000004a10 <.L86>:
    4a10:	ce0594e3          	bnez	a1,46f8 <.L94>
    4a14:	eb5ff06f          	j	48c8 <.L87>

0000000000004a18 <.L38>:
  FP_SUB_Q (R, A, B);
    4a18:	f8048ce3          	beqz	s1,49b0 <.L117>
    4a1c:	00300793          	li	a5,3

0000000000004a20 <.LVL127>:
    4a20:	00f49e63          	bne	s1,a5,4a3c <.L39>
    4a24:	f80906e3          	beqz	s2,49b0 <.L117>

0000000000004a28 <.L116>:
    4a28:	fff00793          	li	a5,-1
    4a2c:	fff00513          	li	a0,-1
    4a30:	ffe70413          	addi	s0,a4,-2

0000000000004a34 <.LBB112>:
  FP_PACK_SEMIRAW_Q (r, R);
    4a34:	00000593          	li	a1,0
    4a38:	fa1ff06f          	j	49d8 <.L204>

0000000000004a3c <.L39>:
  FP_SUB_Q (R, A, B);
    4a3c:	00200793          	li	a5,2
    4a40:	fef494e3          	bne	s1,a5,4a28 <.L116>
    4a44:	f60916e3          	bnez	s2,49b0 <.L117>
    4a48:	fe1ff06f          	j	4a28 <.L116>

0000000000004a4c <.L4>:
    4a4c:	0eb05663          	blez	a1,4b38 <.L40>

0000000000004a50 <.LVL131>:
    4a50:	08089a63          	bnez	a7,4ae4 <.L41>
    4a54:	00c6e5b3          	or	a1,a3,a2
    4a58:	b00588e3          	beqz	a1,4568 <.L224>
    4a5c:	fff7059b          	addiw	a1,a4,-1

0000000000004a60 <.LVL132>:
    4a60:	02059a63          	bnez	a1,4a94 <.L43>

0000000000004a64 <.LBB43>:
    4a64:	40c50633          	sub	a2,a0,a2

0000000000004a68 <.LVL133>:
    4a68:	40d786b3          	sub	a3,a5,a3

0000000000004a6c <.LVL134>:
    4a6c:	00c537b3          	sltu	a5,a0,a2

0000000000004a70 <.LVL135>:
    4a70:	40f687b3          	sub	a5,a3,a5

0000000000004a74 <.LBB44>:
    4a74:	00060513          	mv	a0,a2

0000000000004a78 <.L44>:
    4a78:	0337d713          	srli	a4,a5,0x33
    4a7c:	00177713          	andi	a4,a4,1
    4a80:	44070063          	beqz	a4,4ec0 <.L79>

0000000000004a84 <.LBB45>:
    4a84:	00d79993          	slli	s3,a5,0xd
    4a88:	00d9d993          	srli	s3,s3,0xd

0000000000004a8c <.LVL138>:
    4a8c:	00050a13          	mv	s4,a0
    4a90:	34c0006f          	j	4ddc <.L71>

0000000000004a94 <.L43>:
    4a94:	b7d800e3          	beq	a6,t4,45f4 <.L218>

0000000000004a98 <.L45>:
    4a98:	07400713          	li	a4,116
    4a9c:	08b74863          	blt	a4,a1,4b2c <.L47>
    4aa0:	03f00713          	li	a4,63
    4aa4:	04b74a63          	blt	a4,a1,4af8 <.L48>
    4aa8:	04000813          	li	a6,64
    4aac:	40b8083b          	subw	a6,a6,a1
    4ab0:	01069733          	sll	a4,a3,a6
    4ab4:	00b658b3          	srl	a7,a2,a1

0000000000004ab8 <.LVL141>:
    4ab8:	01061833          	sll	a6,a2,a6
    4abc:	01176733          	or	a4,a4,a7
    4ac0:	01003833          	snez	a6,a6
    4ac4:	01076733          	or	a4,a4,a6

0000000000004ac8 <.LVL142>:
    4ac8:	00b6d5b3          	srl	a1,a3,a1

0000000000004acc <.L49>:
    4acc:	40e50733          	sub	a4,a0,a4

0000000000004ad0 <.LVL144>:
    4ad0:	40b785b3          	sub	a1,a5,a1

0000000000004ad4 <.LVL145>:
    4ad4:	00e537b3          	sltu	a5,a0,a4

0000000000004ad8 <.LVL146>:
    4ad8:	40f587b3          	sub	a5,a1,a5

0000000000004adc <.LBB53>:
    4adc:	00070513          	mv	a0,a4

0000000000004ae0 <.LBE53>:
    4ae0:	f99ff06f          	j	4a78 <.L44>

0000000000004ae4 <.L41>:
    4ae4:	b1d808e3          	beq	a6,t4,45f4 <.L218>
    4ae8:	00100713          	li	a4,1
    4aec:	03371713          	slli	a4,a4,0x33
    4af0:	00e6e6b3          	or	a3,a3,a4
    4af4:	fa5ff06f          	j	4a98 <.L45>

0000000000004af8 <.L48>:
    4af8:	fc05871b          	addiw	a4,a1,-64
    4afc:	04000893          	li	a7,64

0000000000004b00 <.LVL152>:
    4b00:	00e6d733          	srl	a4,a3,a4
    4b04:	00000813          	li	a6,0
    4b08:	01158863          	beq	a1,a7,4b18 <.L50>
    4b0c:	08000813          	li	a6,128
    4b10:	40b8083b          	subw	a6,a6,a1
    4b14:	01069833          	sll	a6,a3,a6

0000000000004b18 <.L50>:
    4b18:	00c86833          	or	a6,a6,a2
    4b1c:	01003833          	snez	a6,a6
    4b20:	01076733          	or	a4,a4,a6

0000000000004b24 <.L193>:
    4b24:	00000593          	li	a1,0

0000000000004b28 <.LVL154>:
    4b28:	fa5ff06f          	j	4acc <.L49>

0000000000004b2c <.L47>:
    4b2c:	00c6e733          	or	a4,a3,a2
    4b30:	00e03733          	snez	a4,a4
    4b34:	ff1ff06f          	j	4b24 <.L193>

0000000000004b38 <.L40>:
    4b38:	12058a63          	beqz	a1,4c6c <.L51>

0000000000004b3c <.LVL157>:
    4b3c:	08081a63          	bnez	a6,4bd0 <.L52>
    4b40:	00a7e5b3          	or	a1,a5,a0
    4b44:	02059a63          	bnez	a1,4b78 <.L53>
    4b48:	39d89863          	bne	a7,t4,4ed8 <.L122>
    4b4c:	00c6e533          	or	a0,a3,a2

0000000000004b50 <.LVL158>:
    4b50:	40050463          	beqz	a0,4f58 <.L125>
    4b54:	0326d713          	srli	a4,a3,0x32

0000000000004b58 <.LVL159>:
    4b58:	00174713          	xori	a4,a4,1
    4b5c:	00177713          	andi	a4,a4,1
    4b60:	00471713          	slli	a4,a4,0x4

0000000000004b64 <.LBE114>:
  FP_UNPACK_SEMIRAW_Q (B, b);
    4b64:	00068793          	mv	a5,a3

0000000000004b68 <.LVL160>:
    4b68:	00060513          	mv	a0,a2

0000000000004b6c <.LBB116>:
    4b6c:	00088413          	mv	s0,a7
    4b70:	00030913          	mv	s2,t1

0000000000004b74 <.LVL161>:
    4b74:	b71ff06f          	j	46e4 <.L9>

0000000000004b78 <.L53>:
  FP_SUB_Q (R, A, B);
    4b78:	fff74713          	not	a4,a4

0000000000004b7c <.LVL163>:
    4b7c:	0007071b          	sext.w	a4,a4

0000000000004b80 <.LVL164>:
    4b80:	02071063          	bnez	a4,4ba0 <.L54>

0000000000004b84 <.LBB55>:
    4b84:	40a60533          	sub	a0,a2,a0

0000000000004b88 <.LVL165>:
    4b88:	40f686b3          	sub	a3,a3,a5

0000000000004b8c <.LVL166>:
    4b8c:	00a63633          	sltu	a2,a2,a0

0000000000004b90 <.LVL167>:
    4b90:	40c687b3          	sub	a5,a3,a2

0000000000004b94 <.L195>:
  FP_UNPACK_SEMIRAW_Q (B, b);
    4b94:	00088413          	mv	s0,a7

0000000000004b98 <.LBB118>:
    4b98:	00030913          	mv	s2,t1

0000000000004b9c <.LVL169>:
    4b9c:	eddff06f          	j	4a78 <.L44>

0000000000004ba0 <.L54>:
  FP_SUB_Q (R, A, B);
    4ba0:	05d89263          	bne	a7,t4,4be4 <.L55>

0000000000004ba4 <.L222>:
    4ba4:	00c6e533          	or	a0,a3,a2

0000000000004ba8 <.LVL172>:
    4ba8:	3a050863          	beqz	a0,4f58 <.L125>
    4bac:	0326d713          	srli	a4,a3,0x32
    4bb0:	00174713          	xori	a4,a4,1
    4bb4:	00177713          	andi	a4,a4,1
    4bb8:	00471713          	slli	a4,a4,0x4

0000000000004bbc <.LBE118>:
  FP_UNPACK_SEMIRAW_Q (B, b);
    4bbc:	00068793          	mv	a5,a3

0000000000004bc0 <.LVL173>:
    4bc0:	00060513          	mv	a0,a2

0000000000004bc4 <.LBB120>:
  FP_SUB_Q (R, A, B);
    4bc4:	00088413          	mv	s0,a7
    4bc8:	00030913          	mv	s2,t1

0000000000004bcc <.LVL174>:
    4bcc:	b81ff06f          	j	474c <.L202>

0000000000004bd0 <.L52>:
    4bd0:	fdd88ae3          	beq	a7,t4,4ba4 <.L222>
    4bd4:	00100593          	li	a1,1
    4bd8:	03359593          	slli	a1,a1,0x33
    4bdc:	40e0073b          	negw	a4,a4

0000000000004be0 <.LVL176>:
    4be0:	00b7e7b3          	or	a5,a5,a1

0000000000004be4 <.L55>:
    4be4:	07400593          	li	a1,116
    4be8:	06e5cc63          	blt	a1,a4,4c60 <.L57>
    4bec:	03f00593          	li	a1,63
    4bf0:	02e5ce63          	blt	a1,a4,4c2c <.L58>
    4bf4:	04000813          	li	a6,64
    4bf8:	40e8083b          	subw	a6,a6,a4
    4bfc:	010795b3          	sll	a1,a5,a6
    4c00:	00e55e33          	srl	t3,a0,a4
    4c04:	01051833          	sll	a6,a0,a6
    4c08:	01c5e5b3          	or	a1,a1,t3
    4c0c:	01003833          	snez	a6,a6
    4c10:	0105e5b3          	or	a1,a1,a6

0000000000004c14 <.LVL178>:
    4c14:	00e7d733          	srl	a4,a5,a4

0000000000004c18 <.L59>:
    4c18:	40b60533          	sub	a0,a2,a1

0000000000004c1c <.LVL180>:
    4c1c:	40e68733          	sub	a4,a3,a4

0000000000004c20 <.LVL181>:
    4c20:	00a63633          	sltu	a2,a2,a0

0000000000004c24 <.LVL182>:
    4c24:	40c707b3          	sub	a5,a4,a2
    4c28:	f6dff06f          	j	4b94 <.L195>

0000000000004c2c <.L58>:
    4c2c:	fc07059b          	addiw	a1,a4,-64
    4c30:	04000e13          	li	t3,64
    4c34:	00b7d5b3          	srl	a1,a5,a1
    4c38:	00000813          	li	a6,0
    4c3c:	01c70863          	beq	a4,t3,4c4c <.L60>
    4c40:	08000813          	li	a6,128
    4c44:	40e8083b          	subw	a6,a6,a4
    4c48:	01079833          	sll	a6,a5,a6

0000000000004c4c <.L60>:
    4c4c:	00a86833          	or	a6,a6,a0
    4c50:	01003833          	snez	a6,a6
    4c54:	0105e5b3          	or	a1,a1,a6

0000000000004c58 <.L194>:
    4c58:	00000713          	li	a4,0

0000000000004c5c <.LVL185>:
    4c5c:	fbdff06f          	j	4c18 <.L59>

0000000000004c60 <.L57>:
    4c60:	00a7e5b3          	or	a1,a5,a0
    4c64:	00b035b3          	snez	a1,a1
    4c68:	ff1ff06f          	j	4c58 <.L194>

0000000000004c6c <.L51>:
    4c6c:	00180f13          	addi	t5,a6,1
    4c70:	ffee0713          	addi	a4,t3,-2
    4c74:	00ef7733          	and	a4,t5,a4
    4c78:	12071a63          	bnez	a4,4dac <.L61>

0000000000004c7c <.LBE120>:
    4c7c:	00c6ee33          	or	t3,a3,a2

0000000000004c80 <.LBB121>:
    4c80:	00a7ef33          	or	t5,a5,a0

0000000000004c84 <.LBB122>:
    4c84:	0c081663          	bnez	a6,4d50 <.L62>

0000000000004c88 <.LVL188>:
    4c88:	060f1c63          	bnez	t5,4d00 <.L63>

0000000000004c8c <.LVL189>:
    4c8c:	260e1663          	bnez	t3,4ef8 <.L127>
    4c90:	ffe48913          	addi	s2,s1,-2

0000000000004c94 <.LVL190>:
    4c94:	00193913          	seqz	s2,s2

0000000000004c98 <.LVL191>:
    4c98:	00000793          	li	a5,0

0000000000004c9c <.LVL192>:
    4c9c:	00000513          	li	a0,0

0000000000004ca0 <.L30>:
  FP_PACK_SEMIRAW_Q (r, R);
    4ca0:	00f56733          	or	a4,a0,a5
    4ca4:	2e070263          	beqz	a4,4f88 <.L134>

0000000000004ca8 <.LBB78>:
    4ca8:	00151693          	slli	a3,a0,0x1
    4cac:	03f55713          	srli	a4,a0,0x3f
    4cb0:	00179593          	slli	a1,a5,0x1
    4cb4:	0076f613          	andi	a2,a3,7
    4cb8:	00e585b3          	add	a1,a1,a4

0000000000004cbc <.LVL195>:
    4cbc:	00000713          	li	a4,0
    4cc0:	02060663          	beqz	a2,4cec <.L80>

0000000000004cc4 <.LVL196>:
    4cc4:	00200713          	li	a4,2
    4cc8:	26e48463          	beq	s1,a4,4f30 <.L81>
    4ccc:	00300713          	li	a4,3
    4cd0:	24e48063          	beq	s1,a4,4f10 <.L82>
    4cd4:	00100713          	li	a4,1
    4cd8:	00049a63          	bnez	s1,4cec <.L80>
    4cdc:	00f6f613          	andi	a2,a3,15
    4ce0:	00400813          	li	a6,4

0000000000004ce4 <.LBB81>:
    4ce4:	ffc6b693          	sltiu	a3,a3,-4

0000000000004ce8 <.LBE81>:
    4ce8:	23061a63          	bne	a2,a6,4f1c <.L198>

0000000000004cec <.L80>:
    4cec:	0345d593          	srli	a1,a1,0x34

0000000000004cf0 <.LVL199>:
    4cf0:	0015c593          	xori	a1,a1,1

0000000000004cf4 <.LBE78>:
    4cf4:	0015f593          	andi	a1,a1,1

0000000000004cf8 <.LBB83>:
    4cf8:	00000413          	li	s0,0
    4cfc:	9e9ff06f          	j	46e4 <.L9>

0000000000004d00 <.L63>:
  FP_SUB_Q (R, A, B);
    4d00:	fa0e00e3          	beqz	t3,4ca0 <.L30>

0000000000004d04 <.LBB60>:
    4d04:	40c505b3          	sub	a1,a0,a2

0000000000004d08 <.LVL201>:
    4d08:	00b53833          	sltu	a6,a0,a1
    4d0c:	40d78733          	sub	a4,a5,a3
    4d10:	41070733          	sub	a4,a4,a6

0000000000004d14 <.LBE60>:
    4d14:	03375813          	srli	a6,a4,0x33
    4d18:	00187813          	andi	a6,a6,1
    4d1c:	00080e63          	beqz	a6,4d38 <.L64>

0000000000004d20 <.LBB61>:
    4d20:	40a60533          	sub	a0,a2,a0

0000000000004d24 <.LVL203>:
    4d24:	40f686b3          	sub	a3,a3,a5

0000000000004d28 <.LVL204>:
    4d28:	00a63633          	sltu	a2,a2,a0

0000000000004d2c <.LVL205>:
    4d2c:	40c687b3          	sub	a5,a3,a2

0000000000004d30 <.L196>:
    4d30:	00030913          	mv	s2,t1

0000000000004d34 <.LVL207>:
    4d34:	f6dff06f          	j	4ca0 <.L30>

0000000000004d38 <.L64>:
    4d38:	00e5e533          	or	a0,a1,a4

0000000000004d3c <.LVL209>:
    4d3c:	1c051463          	bnez	a0,4f04 <.L128>

0000000000004d40 <.L212>:
    4d40:	ffe48913          	addi	s2,s1,-2

0000000000004d44 <.LVL211>:
    4d44:	00193913          	seqz	s2,s2

0000000000004d48 <.L197>:
    4d48:	00000793          	li	a5,0
    4d4c:	f55ff06f          	j	4ca0 <.L30>

0000000000004d50 <.L62>:
    4d50:	03d81a63          	bne	a6,t4,4d84 <.L65>
    4d54:	280f0863          	beqz	t5,4fe4 <.L66>
    4d58:	0327d713          	srli	a4,a5,0x32
    4d5c:	00174713          	xori	a4,a4,1
    4d60:	00177713          	andi	a4,a4,1
    4d64:	00471713          	slli	a4,a4,0x4

0000000000004d68 <.LVL214>:
    4d68:	03089e63          	bne	a7,a6,4da4 <.L68>

0000000000004d6c <.L67>:
    4d6c:	020e0063          	beqz	t3,4d8c <.L69>
    4d70:	0326d813          	srli	a6,a3,0x32
    4d74:	00187813          	andi	a6,a6,1
    4d78:	00081a63          	bnez	a6,4d8c <.L69>
    4d7c:	01000713          	li	a4,16
    4d80:	00c0006f          	j	4d8c <.L69>

0000000000004d84 <.L65>:
  FP_DECL_EX;
    4d84:	00000713          	li	a4,0

0000000000004d88 <.LBB125>:
  FP_SUB_Q (R, A, B);
    4d88:	ffd882e3          	beq	a7,t4,4d6c <.L67>

0000000000004d8c <.L69>:
    4d8c:	000f1c63          	bnez	t5,4da4 <.L68>

0000000000004d90 <.L95>:
    4d90:	1c0e0c63          	beqz	t3,4f68 <.L130>

0000000000004d94 <.LBE125>:
  FP_UNPACK_SEMIRAW_Q (B, b);
    4d94:	00068793          	mv	a5,a3

0000000000004d98 <.LVL219>:
    4d98:	00060513          	mv	a0,a2

0000000000004d9c <.LBB127>:
  FP_SUB_Q (R, A, B);
    4d9c:	00030913          	mv	s2,t1

0000000000004da0 <.LVL221>:
    4da0:	afdff06f          	j	489c <.L131>

0000000000004da4 <.L68>:
    4da4:	ae0e0ce3          	beqz	t3,489c <.L131>
    4da8:	b09ff06f          	j	48b0 <.L205>

0000000000004dac <.L61>:
    4dac:	40c50a33          	sub	s4,a0,a2

0000000000004db0 <.LVL224>:
    4db0:	01453733          	sltu	a4,a0,s4
    4db4:	40d789b3          	sub	s3,a5,a3
    4db8:	40e989b3          	sub	s3,s3,a4

0000000000004dbc <.LBE63>:
    4dbc:	0339d713          	srli	a4,s3,0x33
    4dc0:	00177713          	andi	a4,a4,1
    4dc4:	08070663          	beqz	a4,4e50 <.L70>

0000000000004dc8 <.LBB64>:
    4dc8:	40a60a33          	sub	s4,a2,a0

0000000000004dcc <.LVL226>:
    4dcc:	40f689b3          	sub	s3,a3,a5

0000000000004dd0 <.LVL227>:
    4dd0:	01463633          	sltu	a2,a2,s4

0000000000004dd4 <.LVL228>:
    4dd4:	40c989b3          	sub	s3,s3,a2

0000000000004dd8 <.LBE64>:
    4dd8:	00030913          	mv	s2,t1

0000000000004ddc <.L71>:
    4ddc:	08098063          	beqz	s3,4e5c <.L72>
    4de0:	00098513          	mv	a0,s3
    4de4:	7a4000ef          	jal	ra,5588 <__clzdi2>

0000000000004de8 <.LVL231>:
    4de8:	0005051b          	sext.w	a0,a0

0000000000004dec <.L73>:
    4dec:	ff45069b          	addiw	a3,a0,-12
    4df0:	03f00793          	li	a5,63
    4df4:	00068713          	mv	a4,a3

0000000000004df8 <.LVL233>:
    4df8:	06d7ca63          	blt	a5,a3,4e6c <.L74>
    4dfc:	04000793          	li	a5,64
    4e00:	40d787bb          	subw	a5,a5,a3
    4e04:	00d999b3          	sll	s3,s3,a3

0000000000004e08 <.LVL234>:
    4e08:	00fa57b3          	srl	a5,s4,a5
    4e0c:	0137e7b3          	or	a5,a5,s3

0000000000004e10 <.LVL235>:
    4e10:	00da1533          	sll	a0,s4,a3

0000000000004e14 <.L75>:
    4e14:	0886cc63          	blt	a3,s0,4eac <.L76>
    4e18:	4087073b          	subw	a4,a4,s0
    4e1c:	0017061b          	addiw	a2,a4,1

0000000000004e20 <.LVL237>:
    4e20:	03f00693          	li	a3,63
    4e24:	04c6cc63          	blt	a3,a2,4e7c <.L77>
    4e28:	04000713          	li	a4,64
    4e2c:	40c7073b          	subw	a4,a4,a2
    4e30:	00e796b3          	sll	a3,a5,a4
    4e34:	00c555b3          	srl	a1,a0,a2
    4e38:	00e51733          	sll	a4,a0,a4
    4e3c:	00b6e6b3          	or	a3,a3,a1
    4e40:	00e03733          	snez	a4,a4
    4e44:	00e6e533          	or	a0,a3,a4
    4e48:	00c7d7b3          	srl	a5,a5,a2
    4e4c:	e55ff06f          	j	4ca0 <.L30>

0000000000004e50 <.L70>:
    4e50:	013a6533          	or	a0,s4,s3

0000000000004e54 <.LVL241>:
    4e54:	f80514e3          	bnez	a0,4ddc <.L71>
    4e58:	ee9ff06f          	j	4d40 <.L212>

0000000000004e5c <.L72>:
    4e5c:	000a0513          	mv	a0,s4
    4e60:	728000ef          	jal	ra,5588 <__clzdi2>

0000000000004e64 <.LVL243>:
    4e64:	0405051b          	addiw	a0,a0,64

0000000000004e68 <.LVL244>:
    4e68:	f85ff06f          	j	4dec <.L73>

0000000000004e6c <.L74>:
    4e6c:	fb45079b          	addiw	a5,a0,-76
    4e70:	00fa17b3          	sll	a5,s4,a5

0000000000004e74 <.LVL246>:
    4e74:	00000513          	li	a0,0
    4e78:	f9dff06f          	j	4e14 <.L75>

0000000000004e7c <.L77>:
    4e7c:	fc17071b          	addiw	a4,a4,-63
    4e80:	04000813          	li	a6,64
    4e84:	00e7d6b3          	srl	a3,a5,a4
    4e88:	00000713          	li	a4,0
    4e8c:	01060863          	beq	a2,a6,4e9c <.L78>
    4e90:	08000713          	li	a4,128
    4e94:	40c7073b          	subw	a4,a4,a2
    4e98:	00e79733          	sll	a4,a5,a4

0000000000004e9c <.L78>:
    4e9c:	00e56733          	or	a4,a0,a4
    4ea0:	00e03733          	snez	a4,a4
    4ea4:	00e6e533          	or	a0,a3,a4

0000000000004ea8 <.LVL248>:
    4ea8:	ea1ff06f          	j	4d48 <.L197>

0000000000004eac <.L76>:
    4eac:	fff00713          	li	a4,-1
    4eb0:	03371713          	slli	a4,a4,0x33
    4eb4:	fff70713          	addi	a4,a4,-1
    4eb8:	40d40433          	sub	s0,s0,a3
    4ebc:	00e7f7b3          	and	a5,a5,a4

0000000000004ec0 <.L79>:
  FP_PACK_SEMIRAW_Q (r, R);
    4ec0:	de0400e3          	beqz	s0,4ca0 <.L30>
    4ec4:	f24ff06f          	j	45e8 <.L133>

0000000000004ec8 <.L101>:
  FP_UNPACK_SEMIRAW_Q (B, b);
    4ec8:	00068793          	mv	a5,a3

0000000000004ecc <.LVL253>:
    4ecc:	00060513          	mv	a0,a2

0000000000004ed0 <.LBB130>:
    4ed0:	00088413          	mv	s0,a7
    4ed4:	fedff06f          	j	4ec0 <.L79>

0000000000004ed8 <.L122>:
    4ed8:	00068793          	mv	a5,a3

0000000000004edc <.LVL256>:
    4edc:	00060513          	mv	a0,a2

0000000000004ee0 <.LBB132>:
    4ee0:	00088413          	mv	s0,a7
    4ee4:	00030913          	mv	s2,t1

0000000000004ee8 <.LVL258>:
    4ee8:	fd9ff06f          	j	4ec0 <.L79>

0000000000004eec <.L106>:
    4eec:	00068793          	mv	a5,a3

0000000000004ef0 <.L107>:
  FP_SUB_Q (R, A, B);
    4ef0:	00060513          	mv	a0,a2

0000000000004ef4 <.LVL261>:
    4ef4:	dadff06f          	j	4ca0 <.L30>

0000000000004ef8 <.L127>:
  FP_UNPACK_SEMIRAW_Q (B, b);
    4ef8:	00068793          	mv	a5,a3

0000000000004efc <.LVL263>:
    4efc:	00060513          	mv	a0,a2

0000000000004f00 <.LVL264>:
    4f00:	e31ff06f          	j	4d30 <.L196>

0000000000004f04 <.L128>:
    4f04:	00070793          	mv	a5,a4

0000000000004f08 <.LBB67>:
  FP_SUB_Q (R, A, B);
    4f08:	00058513          	mv	a0,a1
    4f0c:	d95ff06f          	j	4ca0 <.L30>

0000000000004f10 <.L82>:
  FP_PACK_SEMIRAW_Q (r, R);
    4f10:	00100713          	li	a4,1
    4f14:	dc091ce3          	bnez	s2,4cec <.L80>

0000000000004f18 <.L199>:
    4f18:	ff86b693          	sltiu	a3,a3,-8

0000000000004f1c <.L198>:
    4f1c:	0016c693          	xori	a3,a3,1
    4f20:	02069693          	slli	a3,a3,0x20
    4f24:	0206d693          	srli	a3,a3,0x20
    4f28:	00d585b3          	add	a1,a1,a3

0000000000004f2c <.LVL270>:
    4f2c:	dc1ff06f          	j	4cec <.L80>

0000000000004f30 <.L81>:
    4f30:	00100713          	li	a4,1
    4f34:	da090ce3          	beqz	s2,4cec <.L80>
    4f38:	fe1ff06f          	j	4f18 <.L199>

0000000000004f3c <.L104>:
    4f3c:	00000793          	li	a5,0

0000000000004f40 <.LVL273>:
  FP_SUB_Q (R, A, B);
    4f40:	00088413          	mv	s0,a7

0000000000004f44 <.L207>:
    4f44:	00000713          	li	a4,0
    4f48:	981ff06f          	j	48c8 <.L87>

0000000000004f4c <.L120>:
    4f4c:	00000793          	li	a5,0

0000000000004f50 <.LVL276>:
    4f50:	00000513          	li	a0,0

0000000000004f54 <.LVL277>:
    4f54:	ff1ff06f          	j	4f44 <.L207>

0000000000004f58 <.L125>:
    4f58:	00000793          	li	a5,0

0000000000004f5c <.LVL279>:
    4f5c:	00088413          	mv	s0,a7
    4f60:	00030913          	mv	s2,t1

0000000000004f64 <.LVL280>:
    4f64:	fe1ff06f          	j	4f44 <.L207>

0000000000004f68 <.L130>:
    4f68:	00100793          	li	a5,1

0000000000004f6c <.LVL282>:
    4f6c:	00008437          	lui	s0,0x8
    4f70:	00000513          	li	a0,0

0000000000004f74 <.LVL283>:
    4f74:	00000913          	li	s2,0

0000000000004f78 <.LVL284>:
    4f78:	03279793          	slli	a5,a5,0x32
    4f7c:	fff40413          	addi	s0,s0,-1 # 7fff <.LLST3+0xd81>
    4f80:	01000713          	li	a4,16

0000000000004f84 <.LVL285>:
    4f84:	945ff06f          	j	48c8 <.L87>

0000000000004f88 <.L134>:
    4f88:	00000793          	li	a5,0

0000000000004f8c <.LVL287>:
    4f8c:	00000513          	li	a0,0

0000000000004f90 <.LVL288>:
    4f90:	00000413          	li	s0,0
    4f94:	fb1ff06f          	j	4f44 <.L207>

0000000000004f98 <.L85>:
  FP_PACK_SEMIRAW_Q (r, R);
    4f98:	a6091ce3          	bnez	s2,4a10 <.L86>

0000000000004f9c <.L209>:
    4f9c:	00850693          	addi	a3,a0,8
    4fa0:	a65ff06f          	j	4a04 <.L208>

0000000000004fa4 <.L84>:
    4fa4:	a60906e3          	beqz	s2,4a10 <.L86>
    4fa8:	ff5ff06f          	j	4f9c <.L209>

0000000000004fac <.L89>:
    4fac:	00000513          	li	a0,0
    4fb0:	02048463          	beqz	s1,4fd8 <.L90>
    4fb4:	00300793          	li	a5,3

0000000000004fb8 <.LVL291>:
    4fb8:	00f49a63          	bne	s1,a5,4fcc <.L91>
    4fbc:	00090e63          	beqz	s2,4fd8 <.L90>

0000000000004fc0 <.L142>:
    4fc0:	fff00513          	li	a0,-1
    4fc4:	ffe68413          	addi	s0,a3,-2

0000000000004fc8 <.LVL292>:
    4fc8:	0100006f          	j	4fd8 <.L90>

0000000000004fcc <.L91>:
    4fcc:	00200793          	li	a5,2
    4fd0:	fef498e3          	bne	s1,a5,4fc0 <.L142>
    4fd4:	fe0906e3          	beqz	s2,4fc0 <.L142>

0000000000004fd8 <.L90>:
    4fd8:	00576713          	ori	a4,a4,5

0000000000004fdc <.LVL295>:
    4fdc:	00050793          	mv	a5,a0
    4fe0:	915ff06f          	j	48f4 <.L88>

0000000000004fe4 <.L66>:
  FP_DECL_EX;
    4fe4:	00000713          	li	a4,0

0000000000004fe8 <.LBB143>:
  FP_SUB_Q (R, A, B);
    4fe8:	db0894e3          	bne	a7,a6,4d90 <.L95>
    4fec:	d81ff06f          	j	4d6c <.L67>

0000000000004ff0 <.L32>:
    4ff0:	00000713          	li	a4,0
    4ff4:	8b0890e3          	bne	a7,a6,4894 <.L96>
    4ff8:	875ff06f          	j	486c <.L33>

0000000000004ffc <__fixunstfdi>:
#include "soft-fp.h"
#include "quad.h"

UDItype
__fixunstfdi (TFtype a)
{
    4ffc:	00050813          	mv	a6,a0
  FP_DECL_EX;
  FP_DECL_Q (A);
  UDItype r;

  FP_INIT_EXCEPTIONS;
    5000:	002027f3          	frrm	a5

0000000000005004 <.LBB2>:
  FP_UNPACK_RAW_Q (A, a);
  FP_TO_INT_Q (r, A, DI_BITS, 0);
    5004:	000046b7          	lui	a3,0x4

0000000000005008 <.LBB3>:
  FP_UNPACK_RAW_Q (A, a);
    5008:	01059793          	slli	a5,a1,0x10
    500c:	00159713          	slli	a4,a1,0x1
    5010:	0107d513          	srli	a0,a5,0x10

0000000000005014 <.LVL2>:
    5014:	03175713          	srli	a4,a4,0x31

0000000000005018 <.LBE3>:
  FP_TO_INT_Q (r, A, DI_BITS, 0);
    5018:	ffe68793          	addi	a5,a3,-2 # 3ffe <.L29+0x6>

000000000000501c <.LBB4>:
  FP_UNPACK_RAW_Q (A, a);
    501c:	03f5d593          	srli	a1,a1,0x3f

0000000000005020 <.LBE4>:
  FP_TO_INT_Q (r, A, DI_BITS, 0);
    5020:	00e7cc63          	blt	a5,a4,5038 <.L2>

0000000000005024 <.LVL5>:
    5024:	00071663          	bnez	a4,5030 <.L10>
    5028:	00a86533          	or	a0,a6,a0

000000000000502c <.LVL6>:
  FP_HANDLE_EXCEPTIONS;
    502c:	0a050663          	beqz	a0,50d8 <.L1>

0000000000005030 <.L10>:
  FP_TO_INT_Q (r, A, DI_BITS, 0);
    5030:	00000513          	li	a0,0
    5034:	0580006f          	j	508c <.L12>

0000000000005038 <.L2>:
    5038:	03f68793          	addi	a5,a3,63
    503c:	40b787b3          	sub	a5,a5,a1
    5040:	04f75a63          	bge	a4,a5,5094 <.L5>
    5044:	08059c63          	bnez	a1,50dc <.L11>

0000000000005048 <.LBB5>:
    5048:	00100793          	li	a5,1
    504c:	06f68693          	addi	a3,a3,111
    5050:	03079793          	slli	a5,a5,0x30
    5054:	40e68633          	sub	a2,a3,a4
    5058:	03f00593          	li	a1,63

000000000000505c <.LVL10>:
    505c:	00f567b3          	or	a5,a0,a5

0000000000005060 <.LVL11>:
    5060:	0007071b          	sext.w	a4,a4

0000000000005064 <.LVL12>:
    5064:	04c5c063          	blt	a1,a2,50a4 <.L7>
    5068:	ffffc637          	lui	a2,0xffffc
    506c:	fd16061b          	addiw	a2,a2,-47 # ffffffffffffbfd1 <_ebss+0xffffffffffff33a1>
    5070:	00e6063b          	addw	a2,a2,a4

0000000000005074 <.LVL13>:
    5074:	40e686bb          	subw	a3,a3,a4
    5078:	00c79533          	sll	a0,a5,a2
    507c:	00d856b3          	srl	a3,a6,a3
    5080:	00c81633          	sll	a2,a6,a2
    5084:	00a6e533          	or	a0,a3,a0

0000000000005088 <.LBE5>:
  FP_HANDLE_EXCEPTIONS;
    5088:	04060863          	beqz	a2,50d8 <.L1>

000000000000508c <.L12>:
    508c:	00100793          	li	a5,1
    5090:	00c0006f          	j	509c <.L3>

0000000000005094 <.L5>:
  FP_TO_INT_Q (r, A, DI_BITS, 0);
    5094:	fff58513          	addi	a0,a1,-1

0000000000005098 <.L18>:
    5098:	01000793          	li	a5,16

000000000000509c <.L3>:
  FP_HANDLE_EXCEPTIONS;
    509c:	0017a073          	csrs	fflags,a5

  return r;
    50a0:	0380006f          	j	50d8 <.L1>

00000000000050a4 <.L7>:
  FP_TO_INT_Q (r, A, DI_BITS, 0);
    50a4:	04000593          	li	a1,64
    50a8:	00000693          	li	a3,0
    50ac:	00b60a63          	beq	a2,a1,50c0 <.L8>
    50b0:	ffffc6b7          	lui	a3,0xffffc
    50b4:	0116869b          	addiw	a3,a3,17 # ffffffffffffc011 <_ebss+0xffffffffffff33e1>
    50b8:	00d706bb          	addw	a3,a4,a3
    50bc:	00d796b3          	sll	a3,a5,a3

00000000000050c0 <.L8>:
    50c0:	00004637          	lui	a2,0x4
    50c4:	02f6061b          	addiw	a2,a2,47 # 402f <.LVL115+0x2b>
    50c8:	40e6073b          	subw	a4,a2,a4
    50cc:	0106e6b3          	or	a3,a3,a6

00000000000050d0 <.LVL21>:
    50d0:	00e7d533          	srl	a0,a5,a4

00000000000050d4 <.LBE9>:
  FP_HANDLE_EXCEPTIONS;
    50d4:	fa069ce3          	bnez	a3,508c <.L12>

00000000000050d8 <.L1>:
}
    50d8:	00008067          	ret

00000000000050dc <.L11>:
  FP_TO_INT_Q (r, A, DI_BITS, 0);
    50dc:	00000513          	li	a0,0

00000000000050e0 <.LVL25>:
    50e0:	fb9ff06f          	j	5098 <.L18>

00000000000050e4 <__floatunditf>:
#include "soft-fp.h"
#include "quad.h"

TFtype
__floatunditf (UDItype i)
{
    50e4:	ff010113          	addi	sp,sp,-16
    50e8:	00813023          	sd	s0,0(sp)
    50ec:	00113423          	sd	ra,8(sp)
    50f0:	00050413          	mv	s0,a0

00000000000050f4 <.LBB9>:
  FP_DECL_Q (A);
  TFtype a;

  FP_FROM_INT_Q (A, i, DI_BITS, UDItype);
    50f4:	06050863          	beqz	a0,5164 <.L4>

00000000000050f8 <.LBB3>:
    50f8:	490000ef          	jal	ra,5588 <__clzdi2>

00000000000050fc <.LVL2>:
    50fc:	000045b7          	lui	a1,0x4
    5100:	03e5859b          	addiw	a1,a1,62 # 403e <.LVL118+0x2>

0000000000005104 <.LBE4>:
    5104:	000047b7          	lui	a5,0x4

0000000000005108 <.LBB5>:
    5108:	40a585bb          	subw	a1,a1,a0

000000000000510c <.LBE5>:
    510c:	06f78793          	addi	a5,a5,111 # 406f <.LBB81+0x3>
    5110:	40b787b3          	sub	a5,a5,a1
    5114:	03f00713          	li	a4,63
    5118:	02f74e63          	blt	a4,a5,5154 <.L3>
    511c:	00f00793          	li	a5,15
    5120:	40a787bb          	subw	a5,a5,a0
    5124:	0315051b          	addiw	a0,a0,49

0000000000005128 <.LVL4>:
    5128:	00f457b3          	srl	a5,s0,a5

000000000000512c <.LVL5>:
    512c:	00a41433          	sll	s0,s0,a0

0000000000005130 <.L2>:
  FP_PACK_RAW_Q (a, A);

  return a;
}
    5130:	00813083          	ld	ra,8(sp)
    5134:	00040513          	mv	a0,s0
    5138:	00013403          	ld	s0,0(sp)

000000000000513c <.LBB11>:
  FP_PACK_RAW_Q (a, A);
    513c:	01079793          	slli	a5,a5,0x10

0000000000005140 <.LVL8>:
    5140:	03059593          	slli	a1,a1,0x30

0000000000005144 <.LVL9>:
    5144:	0107d793          	srli	a5,a5,0x10

0000000000005148 <.LBE11>:
}
    5148:	00b7e5b3          	or	a1,a5,a1
    514c:	01010113          	addi	sp,sp,16
    5150:	00008067          	ret

0000000000005154 <.L3>:
  FP_FROM_INT_Q (A, i, DI_BITS, UDItype);
    5154:	ff15051b          	addiw	a0,a0,-15

0000000000005158 <.LVL11>:
    5158:	00a417b3          	sll	a5,s0,a0

000000000000515c <.LVL12>:
    515c:	00000413          	li	s0,0

0000000000005160 <.LVL13>:
    5160:	fd1ff06f          	j	5130 <.L2>

0000000000005164 <.L4>:
    5164:	00000793          	li	a5,0
    5168:	00000593          	li	a1,0
    516c:	fc5ff06f          	j	5130 <.L2>

0000000000005170 <__extenddftf2>:
#include "double.h"
#include "quad.h"

TFtype
__extenddftf2 (DFtype a)
{
    5170:	fe010113          	addi	sp,sp,-32
    5174:	e20507d3          	fmv.x.d	a5,fa0
    5178:	00113c23          	sd	ra,24(sp)
    517c:	00813823          	sd	s0,16(sp)
    5180:	00913423          	sd	s1,8(sp)
  FP_DECL_EX;
  FP_DECL_D (A);
  FP_DECL_Q (R);
  TFtype r;

  FP_INIT_EXCEPTIONS;
    5184:	00202773          	frrm	a4

0000000000005188 <.LBB2>:
  FP_UNPACK_RAW_D (A, a);
    5188:	0347d513          	srli	a0,a5,0x34
    518c:	7ff57513          	andi	a0,a0,2047
    5190:	00c79413          	slli	s0,a5,0xc
    5194:	03f7d493          	srli	s1,a5,0x3f

0000000000005198 <.LBE2>:
#if _FP_W_TYPE_SIZE < 64
  FP_EXTEND (Q, D, 4, 2, R, A);
#else
  FP_EXTEND (Q, D, 2, 1, R, A);
    5198:	00150793          	addi	a5,a0,1
    519c:	7fe7f793          	andi	a5,a5,2046

00000000000051a0 <.LBB3>:
  FP_UNPACK_RAW_D (A, a);
    51a0:	00c45413          	srli	s0,s0,0xc

00000000000051a4 <.LBE3>:
  FP_EXTEND (Q, D, 2, 1, R, A);
    51a4:	02078063          	beqz	a5,51c4 <.L2>
    51a8:	000047b7          	lui	a5,0x4
    51ac:	c0078793          	addi	a5,a5,-1024 # 3c00 <.L1>
    51b0:	00f50533          	add	a0,a0,a5

00000000000051b4 <.LVL3>:
    51b4:	00445793          	srli	a5,s0,0x4

00000000000051b8 <.LVL4>:
    51b8:	03c41413          	slli	s0,s0,0x3c

00000000000051bc <.L14>:
  FP_DECL_EX;
    51bc:	00000713          	li	a4,0
    51c0:	0880006f          	j	5248 <.L3>

00000000000051c4 <.L2>:
  FP_EXTEND (Q, D, 2, 1, R, A);
    51c4:	04051a63          	bnez	a0,5218 <.L4>
    51c8:	00000793          	li	a5,0
    51cc:	fe0408e3          	beqz	s0,51bc <.L14>

00000000000051d0 <.LBB5>:
    51d0:	00040513          	mv	a0,s0

00000000000051d4 <.LVL7>:
    51d4:	3b4000ef          	jal	ra,5588 <__clzdi2>

00000000000051d8 <.LVL8>:
    51d8:	0005071b          	sext.w	a4,a0

00000000000051dc <.LVL9>:
    51dc:	00e00793          	li	a5,14
    51e0:	02e7c463          	blt	a5,a4,5208 <.L5>
    51e4:	00f00793          	li	a5,15
    51e8:	40a787bb          	subw	a5,a5,a0
    51ec:	0315071b          	addiw	a4,a0,49
    51f0:	00f457b3          	srl	a5,s0,a5

00000000000051f4 <.LVL10>:
    51f4:	00e41433          	sll	s0,s0,a4

00000000000051f8 <.L6>:
    51f8:	00004737          	lui	a4,0x4
    51fc:	c0c7071b          	addiw	a4,a4,-1012 # 3c0c <.L31>
    5200:	40a7053b          	subw	a0,a4,a0

0000000000005204 <.LVL12>:
    5204:	fb9ff06f          	j	51bc <.L14>

0000000000005208 <.L5>:
    5208:	ff15079b          	addiw	a5,a0,-15
    520c:	00f417b3          	sll	a5,s0,a5

0000000000005210 <.LVL14>:
    5210:	00000413          	li	s0,0
    5214:	fe5ff06f          	j	51f8 <.L6>

0000000000005218 <.L4>:
    5218:	06040463          	beqz	s0,5280 <.L9>
    521c:	03345713          	srli	a4,s0,0x33
    5220:	00174713          	xori	a4,a4,1
    5224:	00100693          	li	a3,1
    5228:	00445793          	srli	a5,s0,0x4
    522c:	00177713          	andi	a4,a4,1
    5230:	02f69693          	slli	a3,a3,0x2f
    5234:	00008537          	lui	a0,0x8

0000000000005238 <.LVL16>:
    5238:	00471713          	slli	a4,a4,0x4

000000000000523c <.LVL17>:
    523c:	00d7e7b3          	or	a5,a5,a3
    5240:	03c41413          	slli	s0,s0,0x3c

0000000000005244 <.LVL18>:
    5244:	fff50513          	addi	a0,a0,-1 # 7fff <.LLST3+0xd81>

0000000000005248 <.L3>:
#endif
  FP_PACK_RAW_Q (r, R);
    5248:	01079593          	slli	a1,a5,0x10
    524c:	00f4979b          	slliw	a5,s1,0xf

0000000000005250 <.LVL20>:
    5250:	00f56533          	or	a0,a0,a5

0000000000005254 <.LVL21>:
    5254:	03051513          	slli	a0,a0,0x30
    5258:	0105d593          	srli	a1,a1,0x10
    525c:	00a5e5b3          	or	a1,a1,a0

0000000000005260 <.LBE10>:
  FP_HANDLE_EXCEPTIONS;
    5260:	00070463          	beqz	a4,5268 <.L1>
    5264:	00172073          	csrs	fflags,a4

0000000000005268 <.L1>:

  return r;
}
    5268:	01813083          	ld	ra,24(sp)
    526c:	00040513          	mv	a0,s0
    5270:	01013403          	ld	s0,16(sp)
    5274:	00813483          	ld	s1,8(sp)

0000000000005278 <.LVL23>:
    5278:	02010113          	addi	sp,sp,32
    527c:	00008067          	ret

0000000000005280 <.L9>:
  FP_EXTEND (Q, D, 2, 1, R, A);
    5280:	00008537          	lui	a0,0x8

0000000000005284 <.LVL25>:
    5284:	00000793          	li	a5,0
    5288:	fff50513          	addi	a0,a0,-1 # 7fff <.LLST3+0xd81>
    528c:	f31ff06f          	j	51bc <.L14>

0000000000005290 <__trunctfdf2>:
  FP_DECL_EX;
  FP_DECL_Q (A);
  FP_DECL_D (R);
  DFtype r;

  FP_INIT_ROUNDMODE;
    5290:	00202673          	frrm	a2

0000000000005294 <.LBB2>:
  FP_UNPACK_SEMIRAW_Q (A, a);
    5294:	000088b7          	lui	a7,0x8
    5298:	0305d713          	srli	a4,a1,0x30
    529c:	fff88e13          	addi	t3,a7,-1 # 7fff <.LLST3+0xd81>
    52a0:	01c77833          	and	a6,a4,t3
    52a4:	03f5d693          	srli	a3,a1,0x3f

00000000000052a8 <.LBE2>:
#if _FP_W_TYPE_SIZE < 64
  FP_TRUNC (D, Q, 2, 4, R, A);
#else
  FP_TRUNC (D, Q, 1, 2, R, A);
    52a8:	00180793          	addi	a5,a6,1

00000000000052ac <.LBB3>:
  FP_UNPACK_SEMIRAW_Q (A, a);
    52ac:	01059593          	slli	a1,a1,0x10

00000000000052b0 <.LBE3>:
  FP_TRUNC (D, Q, 1, 2, R, A);
    52b0:	ffe88893          	addi	a7,a7,-2
  FP_UNPACK_SEMIRAW_Q (A, a);
    52b4:	00d5d593          	srli	a1,a1,0xd
    52b8:	03d55713          	srli	a4,a0,0x3d
  FP_TRUNC (D, Q, 1, 2, R, A);
    52bc:	0117f7b3          	and	a5,a5,a7
  FP_INIT_ROUNDMODE;
    52c0:	0006061b          	sext.w	a2,a2

00000000000052c4 <.LBB5>:
  FP_UNPACK_SEMIRAW_Q (A, a);
    52c4:	00b76733          	or	a4,a4,a1

00000000000052c8 <.LVL2>:
    52c8:	00351313          	slli	t1,a0,0x3

00000000000052cc <.LVL3>:
  FP_TRUNC (D, Q, 1, 2, R, A);
    52cc:	1e078a63          	beqz	a5,54c0 <.L2>
    52d0:	ffffc7b7          	lui	a5,0xffffc
    52d4:	40078793          	addi	a5,a5,1024 # ffffffffffffc400 <_ebss+0xffffffffffff37d0>
    52d8:	00f80833          	add	a6,a6,a5

00000000000052dc <.LVL4>:
    52dc:	7fe00793          	li	a5,2046
    52e0:	0f07d063          	bge	a5,a6,53c0 <.L3>
    52e4:	06060063          	beqz	a2,5344 <.L31>
    52e8:	00300793          	li	a5,3
    52ec:	04f61663          	bne	a2,a5,5338 <.L5>
    52f0:	04068a63          	beqz	a3,5344 <.L31>

00000000000052f4 <.L30>:
    52f4:	fff00793          	li	a5,-1
    52f8:	7fe00813          	li	a6,2046

00000000000052fc <.LBB7>:
#endif
  FP_PACK_SEMIRAW_D (r, R);
    52fc:	00000713          	li	a4,0

0000000000005300 <.LBE7>:
  FP_TRUNC (D, Q, 1, 2, R, A);
    5300:	00500593          	li	a1,5

0000000000005304 <.L68>:
  FP_PACK_SEMIRAW_D (r, R);
    5304:	00200513          	li	a0,2
    5308:	0015e593          	ori	a1,a1,1
    530c:	24a60063          	beq	a2,a0,554c <.L17>
    5310:	00300513          	li	a0,3
    5314:	22a60663          	beq	a2,a0,5540 <.L18>
    5318:	00061a63          	bnez	a2,532c <.L71>
    531c:	00f7f513          	andi	a0,a5,15
    5320:	00400893          	li	a7,4
    5324:	01150463          	beq	a0,a7,532c <.L71>
    5328:	00478793          	addi	a5,a5,4

000000000000532c <.L71>:
    532c:	02070263          	beqz	a4,5350 <.L20>
    5330:	0025e593          	ori	a1,a1,2
    5334:	01c0006f          	j	5350 <.L20>

0000000000005338 <.L5>:
  FP_TRUNC (D, Q, 1, 2, R, A);
    5338:	00200793          	li	a5,2
    533c:	faf61ce3          	bne	a2,a5,52f4 <.L30>
    5340:	fa068ae3          	beqz	a3,52f4 <.L30>

0000000000005344 <.L31>:
    5344:	00000793          	li	a5,0
    5348:	7ff00813          	li	a6,2047

000000000000534c <.LVL12>:
    534c:	00500593          	li	a1,5

0000000000005350 <.L20>:
  FP_PACK_SEMIRAW_D (r, R);
    5350:	0377d713          	srli	a4,a5,0x37
    5354:	00177713          	andi	a4,a4,1
    5358:	02070063          	beqz	a4,5378 <.L21>

000000000000535c <.LVL14>:
    535c:	00180813          	addi	a6,a6,1

0000000000005360 <.LVL15>:
    5360:	7ff00713          	li	a4,2047
    5364:	1ee80863          	beq	a6,a4,5554 <.L22>
    5368:	fff00613          	li	a2,-1

000000000000536c <.LVL16>:
    536c:	03761613          	slli	a2,a2,0x37
    5370:	fff60613          	addi	a2,a2,-1
    5374:	00c7f7b3          	and	a5,a5,a2

0000000000005378 <.L21>:
    5378:	7ff00713          	li	a4,2047
    537c:	0037d793          	srli	a5,a5,0x3
    5380:	00e81a63          	bne	a6,a4,5394 <.L25>
    5384:	00078863          	beqz	a5,5394 <.L25>
    5388:	00100793          	li	a5,1

000000000000538c <.LVL19>:
    538c:	03379793          	slli	a5,a5,0x33
    5390:	00000693          	li	a3,0

0000000000005394 <.L25>:
    5394:	7ff87713          	andi	a4,a6,2047
    5398:	00c79793          	slli	a5,a5,0xc

000000000000539c <.LVL21>:
    539c:	03471713          	slli	a4,a4,0x34
    53a0:	00c7d793          	srli	a5,a5,0xc
    53a4:	00f767b3          	or	a5,a4,a5
    53a8:	03f69693          	slli	a3,a3,0x3f

00000000000053ac <.LVL22>:
    53ac:	00d7e7b3          	or	a5,a5,a3

00000000000053b0 <.LBE13>:
  FP_HANDLE_EXCEPTIONS;
    53b0:	00058463          	beqz	a1,53b8 <.L1>
    53b4:	0015a073          	csrs	fflags,a1

00000000000053b8 <.L1>:

  return r;
}
    53b8:	f2078553          	fmv.d.x	fa0,a5
    53bc:	00008067          	ret

00000000000053c0 <.L3>:
  FP_TRUNC (D, Q, 1, 2, R, A);
    53c0:	0d004663          	bgtz	a6,548c <.L6>
    53c4:	fcc00593          	li	a1,-52
    53c8:	00100793          	li	a5,1
    53cc:	04b84263          	blt	a6,a1,5410 <.L7>
    53d0:	03379793          	slli	a5,a5,0x33
    53d4:	03d00893          	li	a7,61
    53d8:	00f76733          	or	a4,a4,a5
    53dc:	410888b3          	sub	a7,a7,a6
    53e0:	03f00793          	li	a5,63
    53e4:	0008051b          	sext.w	a0,a6

00000000000053e8 <.LVL26>:
    53e8:	0717ca63          	blt	a5,a7,545c <.L8>
    53ec:	03d00793          	li	a5,61
    53f0:	0035059b          	addiw	a1,a0,3
    53f4:	40a787bb          	subw	a5,a5,a0
    53f8:	00f357b3          	srl	a5,t1,a5
    53fc:	00b31333          	sll	t1,t1,a1

0000000000005400 <.LVL27>:
    5400:	00603333          	snez	t1,t1
    5404:	0067e7b3          	or	a5,a5,t1
    5408:	00b71733          	sll	a4,a4,a1

000000000000540c <.LVL28>:
    540c:	00f767b3          	or	a5,a4,a5

0000000000005410 <.L7>:
  FP_PACK_SEMIRAW_D (r, R);
    5410:	00179713          	slli	a4,a5,0x1

0000000000005414 <.LVL30>:
    5414:	00777513          	andi	a0,a4,7
    5418:	00000593          	li	a1,0
    541c:	02050663          	beqz	a0,5448 <.L13>

0000000000005420 <.LVL31>:
    5420:	00200593          	li	a1,2
    5424:	10b60463          	beq	a2,a1,552c <.L14>
    5428:	00300593          	li	a1,3
    542c:	0eb60863          	beq	a2,a1,551c <.L15>
    5430:	00100593          	li	a1,1
    5434:	00061a63          	bnez	a2,5448 <.L13>
    5438:	00f77513          	andi	a0,a4,15
    543c:	00400813          	li	a6,4
    5440:	01050463          	beq	a0,a6,5448 <.L13>
    5444:	00470713          	addi	a4,a4,4

0000000000005448 <.L13>:
    5448:	03875713          	srli	a4,a4,0x38

000000000000544c <.LVL33>:
    544c:	00174713          	xori	a4,a4,1

0000000000005450 <.LBE8>:
    5450:	00177713          	andi	a4,a4,1

0000000000005454 <.LBB9>:
    5454:	00000813          	li	a6,0
    5458:	0540006f          	j	54ac <.L4>

000000000000545c <.L8>:
  FP_TRUNC (D, Q, 1, 2, R, A);
    545c:	ffd00793          	li	a5,-3
    5460:	40a787bb          	subw	a5,a5,a0
    5464:	04000813          	li	a6,64

0000000000005468 <.LVL35>:
    5468:	00f757b3          	srl	a5,a4,a5
    546c:	00000593          	li	a1,0
    5470:	01088663          	beq	a7,a6,547c <.L10>
    5474:	0435051b          	addiw	a0,a0,67
    5478:	00a715b3          	sll	a1,a4,a0

000000000000547c <.L10>:
    547c:	0065e5b3          	or	a1,a1,t1
    5480:	00b035b3          	snez	a1,a1
    5484:	00b7e7b3          	or	a5,a5,a1

0000000000005488 <.LVL36>:
    5488:	f89ff06f          	j	5410 <.L7>

000000000000548c <.L6>:
    548c:	00751793          	slli	a5,a0,0x7
    5490:	00f037b3          	snez	a5,a5
    5494:	03c35313          	srli	t1,t1,0x3c

0000000000005498 <.LVL38>:
    5498:	00471713          	slli	a4,a4,0x4

000000000000549c <.LVL39>:
    549c:	0067e7b3          	or	a5,a5,t1
    54a0:	00f767b3          	or	a5,a4,a5

00000000000054a4 <.LBB19>:
  FP_DECL_EX;
    54a4:	00000593          	li	a1,0

00000000000054a8 <.LBB20>:
  FP_PACK_SEMIRAW_D (r, R);
    54a8:	00000713          	li	a4,0

00000000000054ac <.L4>:
    54ac:	0077f513          	andi	a0,a5,7
    54b0:	e4051ae3          	bnez	a0,5304 <.L68>

00000000000054b4 <.LVL42>:
    54b4:	e8070ee3          	beqz	a4,5350 <.L20>
    54b8:	0015f713          	andi	a4,a1,1

00000000000054bc <.LVL43>:
    54bc:	e71ff06f          	j	532c <.L71>

00000000000054c0 <.L2>:
  FP_TRUNC (D, Q, 1, 2, R, A);
    54c0:	006767b3          	or	a5,a4,t1
    54c4:	00081863          	bnez	a6,54d4 <.L11>

00000000000054c8 <.LBB21>:
  FP_PACK_SEMIRAW_D (r, R);
    54c8:	04079663          	bnez	a5,5514 <.L34>

00000000000054cc <.L72>:
  FP_DECL_EX;
    54cc:	00000593          	li	a1,0
    54d0:	e81ff06f          	j	5350 <.L20>

00000000000054d4 <.L11>:
  FP_TRUNC (D, Q, 1, 2, R, A);
    54d4:	06078263          	beqz	a5,5538 <.L35>
  FP_DECL_EX;
    54d8:	00000593          	li	a1,0
  FP_TRUNC (D, Q, 1, 2, R, A);
    54dc:	01c81863          	bne	a6,t3,54ec <.L12>
    54e0:	03275593          	srli	a1,a4,0x32
    54e4:	0015c593          	xori	a1,a1,1
    54e8:	00459593          	slli	a1,a1,0x4

00000000000054ec <.L12>:
    54ec:	03c35793          	srli	a5,t1,0x3c

00000000000054f0 <.LVL49>:
    54f0:	00471713          	slli	a4,a4,0x4

00000000000054f4 <.LVL50>:
    54f4:	00f76733          	or	a4,a4,a5

00000000000054f8 <.LVL51>:
    54f8:	00100793          	li	a5,1
    54fc:	ff877713          	andi	a4,a4,-8

0000000000005500 <.LVL52>:
    5500:	03679793          	slli	a5,a5,0x36
    5504:	00f767b3          	or	a5,a4,a5

0000000000005508 <.LVL53>:
    5508:	7ff00813          	li	a6,2047

000000000000550c <.LBB24>:
  FP_PACK_SEMIRAW_D (r, R);
    550c:	00000713          	li	a4,0
    5510:	f9dff06f          	j	54ac <.L4>

0000000000005514 <.L34>:
  FP_TRUNC (D, Q, 1, 2, R, A);
    5514:	00100793          	li	a5,1

0000000000005518 <.LVL56>:
    5518:	ef9ff06f          	j	5410 <.L7>

000000000000551c <.L15>:
  FP_PACK_SEMIRAW_D (r, R);
    551c:	00100593          	li	a1,1
    5520:	f20694e3          	bnez	a3,5448 <.L13>

0000000000005524 <.L69>:
    5524:	00870713          	addi	a4,a4,8
    5528:	f21ff06f          	j	5448 <.L13>

000000000000552c <.L14>:
    552c:	00100593          	li	a1,1
    5530:	f0068ce3          	beqz	a3,5448 <.L13>
    5534:	ff1ff06f          	j	5524 <.L69>

0000000000005538 <.L35>:
  FP_TRUNC (D, Q, 1, 2, R, A);
    5538:	7ff00813          	li	a6,2047

000000000000553c <.LVL60>:
    553c:	f91ff06f          	j	54cc <.L72>

0000000000005540 <.L18>:
  FP_PACK_SEMIRAW_D (r, R);
    5540:	de0696e3          	bnez	a3,532c <.L71>

0000000000005544 <.L73>:
    5544:	00878793          	addi	a5,a5,8

0000000000005548 <.LVL62>:
    5548:	de5ff06f          	j	532c <.L71>

000000000000554c <.L17>:
    554c:	de0680e3          	beqz	a3,532c <.L71>
    5550:	ff5ff06f          	j	5544 <.L73>

0000000000005554 <.L22>:
    5554:	00000793          	li	a5,0

0000000000005558 <.LVL65>:
    5558:	02060463          	beqz	a2,5580 <.L23>
    555c:	00300713          	li	a4,3
    5560:	00e61a63          	bne	a2,a4,5574 <.L24>
    5564:	00068e63          	beqz	a3,5580 <.L23>

0000000000005568 <.L44>:
    5568:	fff00793          	li	a5,-1
    556c:	7fe00813          	li	a6,2046

0000000000005570 <.LVL66>:
    5570:	0100006f          	j	5580 <.L23>

0000000000005574 <.L24>:
    5574:	00200713          	li	a4,2
    5578:	fee618e3          	bne	a2,a4,5568 <.L44>
    557c:	fe0686e3          	beqz	a3,5568 <.L44>

0000000000005580 <.L23>:
    5580:	0055e593          	ori	a1,a1,5

0000000000005584 <.LVL69>:
    5584:	df5ff06f          	j	5378 <.L21>

0000000000005588 <__clzdi2>:
int
__clzSI2 (UWtype x)
{
  Wtype ret;

  count_leading_zeros (ret, x);
    5588:	03800793          	li	a5,56

000000000000558c <.L3>:
    558c:	00f55733          	srl	a4,a0,a5
    5590:	0ff77713          	zext.b	a4,a4
    5594:	00071663          	bnez	a4,55a0 <.L2>
    5598:	ff878793          	addi	a5,a5,-8
    559c:	fe0798e3          	bnez	a5,558c <.L3>

00000000000055a0 <.L2>:
    55a0:	04000713          	li	a4,64
    55a4:	40f70733          	sub	a4,a4,a5
    55a8:	00f55533          	srl	a0,a0,a5

00000000000055ac <.LVL4>:
    55ac:	00000797          	auipc	a5,0x0
    55b0:	0cc7b783          	ld	a5,204(a5) # 5678 <.LLST14+0x2c>

00000000000055b4 <.LVL5>:
    55b4:	00a787b3          	add	a5,a5,a0
    55b8:	0007c503          	lbu	a0,0(a5)

00000000000055bc <.LBE3>:

  return ret;
}
    55bc:	40a7053b          	subw	a0,a4,a0
    55c0:	00008067          	ret

00000000000055c4 <main>:
#include <stdio.h>

static int initial[] = {1, 2, 3, 4, 6};

int main( int argc, char ** argv )
{
    55c4:	ff010113          	addi	sp,sp,-16
	
	printf( "Hello, world %08x\n", initial[3] );
    55c8:	00400593          	li	a1,4

00000000000055cc <.LVL1>:
    55cc:	00001517          	auipc	a0,0x1
    55d0:	30450513          	addi	a0,a0,772 # 68d0 <.LC0>

00000000000055d4 <.LVL2>:
{
    55d4:	00113423          	sd	ra,8(sp)
	printf( "Hello, world %08x\n", initial[3] );
    55d8:	b49fa0ef          	jal	ra,120 <__GI_printf>

00000000000055dc <.LVL3>:
}
    55dc:	00813083          	ld	ra,8(sp)
    55e0:	00000513          	li	a0,0
    55e4:	01010113          	addi	sp,sp,16
    55e8:	00008067          	ret
