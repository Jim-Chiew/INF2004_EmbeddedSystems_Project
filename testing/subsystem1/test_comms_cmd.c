/*
 * test_comms_cmd.c -- PC test of the START/STOP/RESET command handling in comms.c.
 * The platform and lwIP are replaced by stand-ins in stubs/, so it runs on a PC.
 * Each check prints an ID S1-Dnn in the order it runs. Add new checks at the END.
 * Build and run from the repo root:
 *   gcc -std=c11 -Wall -Wextra -fsanitize=address,undefined -DCOMMS_HOST_TEST -Itesting/subsystem1/stubs \
 *       -Iapp_program/subsystem1 testing/subsystem1/test_comms_cmd.c \
 *       app_program/subsystem1/trans_queue.c app_program/subsystem1/com_state.c \
 *       app_program/subsystem1/status_led.c app_program/subsystem1/telemetry.c -o /tmp/test_comms_cmd
 *   /tmp/test_comms_cmd
 */
#include <stdio.h>
#include <string.h>
#include "fake_port.h"
#include "../../app_program/subsystem1/comms.c"
static int fails=0;
static int checks=0;
#define CHECK(c,m) do{ checks++; if(c) printf("PASS S1-D%02d %s\n",checks,m); else {printf("FAIL S1-D%02d %s\n",checks,m); fails++;} }while(0)
static void msg(const char*topic,const char*payload){
    inpub_cb(NULL,topic,(u32_t)strlen(payload));
    indata_cb(NULL,(const u8_t*)payload,(u16_t)strlen(payload),MQTT_DATA_FLAG_LAST);
}
int main(void){
    CHECK(comms_get_command()==COMMS_CMD_NONE,"empty at start");
    msg(COMMS_TOPIC_CMD,"START"); CHECK(comms_get_command()==COMMS_CMD_START,"START accepted");
    msg(COMMS_TOPIC_CMD,"STOP");  CHECK(comms_get_command()==COMMS_CMD_STOP,"STOP accepted");
    msg(COMMS_TOPIC_CMD,"RESET"); CHECK(comms_get_command()==COMMS_CMD_RESET,"RESET accepted");
    CHECK(comms_get_command()==COMMS_CMD_NONE,"each command returned once");
    uint32_t rej=g_cmds_rejected;
    msg(COMMS_TOPIC_CMD,"start"); msg(COMMS_TOPIC_CMD,"START "); msg(COMMS_TOPIC_CMD,"GO");
    msg(COMMS_TOPIC_CMD,""); msg(COMMS_TOPIC_CMD,"STOPSTOP"); msg(COMMS_TOPIC_CMD,"STOPSTOPS");
    CHECK(comms_get_command()==COMMS_CMD_NONE,"bad text gives no command");
    CHECK(g_cmds_rejected-rej==6,"6 bad messages counted");
    rej=g_cmds_rejected;
    msg("robocar/car1/other","START");
    CHECK(comms_get_command()==COMMS_CMD_NONE,"other topic ignored");
    CHECK(g_cmds_rejected==rej,"other topic not counted as bad command");
    /* fragmented: "ST" + "OP" */
    inpub_cb(NULL,COMMS_TOPIC_CMD,4);
    indata_cb(NULL,(const u8_t*)"ST",2,0);
    indata_cb(NULL,(const u8_t*)"OP",2,MQTT_DATA_FLAG_LAST);
    CHECK(comms_get_command()==COMMS_CMD_STOP,"fragmented STOP accepted");
    /* ring holds 3 (size 4), 4th is dropped and counted */
    uint32_t acc=g_cmds_accepted; rej=g_cmds_rejected;
    msg(COMMS_TOPIC_CMD,"START"); msg(COMMS_TOPIC_CMD,"STOP"); msg(COMMS_TOPIC_CMD,"RESET"); msg(COMMS_TOPIC_CMD,"START");
    CHECK(g_cmds_accepted-acc==3 && g_cmds_rejected-rej==1,"ring full: 3 kept, 1 dropped and counted");
    CHECK(comms_get_command()==COMMS_CMD_START && comms_get_command()==COMMS_CMD_STOP && comms_get_command()==COMMS_CMD_RESET,"kept in order");
    CHECK(comms_get_command()==COMMS_CMD_NONE,"then empty");
    /* texts of the right LENGTH but the wrong letters must not pass (added later: IDs above unchanged) */
    rej=g_cmds_rejected; acc=g_cmds_accepted;
    msg(COMMS_TOPIC_CMD,"STOQ"); msg(COMMS_TOPIC_CMD,"stop"); msg(COMMS_TOPIC_CMD,"GOGO");
    msg(COMMS_TOPIC_CMD,"STARX"); msg(COMMS_TOPIC_CMD,"RESEX"); msg(COMMS_TOPIC_CMD,"Reset"); msg(COMMS_TOPIC_CMD,"START\n");
    CHECK(comms_get_command()==COMMS_CMD_NONE && g_cmds_accepted==acc,"same-length wrong texts give no command");
    CHECK(g_cmds_rejected-rej==7,"7 wrong texts counted as rejected");
    printf("%d checks, %s (%d failed)\n", checks, fails?"FAILED":"ALL PASSED", fails);
    return fails!=0;
}