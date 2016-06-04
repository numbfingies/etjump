#include <string>
#include <boost/format.hpp>

extern "C" {
#include "cg_local.h"
}

#include "etj_timerun_view.hpp"

void ETJump::TimerunView::start()
{
	auto clientNum = atoi(CG_Argv(2));
	_playersTimerunInformation[clientNum].startTime = atoi(CG_Argv(3));
	_playersTimerunInformation[clientNum].runName = CG_Argv(4);
	_playersTimerunInformation[clientNum].previousRecord = atoi(CG_Argv(5));
	_playersTimerunInformation[clientNum].running = true;
}

void ETJump::TimerunView::stop()
{
	auto clientNum = atoi(CG_Argv(2));
	_playersTimerunInformation[clientNum].completionTime = atoi(CG_Argv(3));
	_playersTimerunInformation[clientNum].running = false;
	_playersTimerunInformation[clientNum].lastRunTimer = cg.time;
}

void ETJump::TimerunView::interrupt(PlayerTimerunInformation& playerTimerunInformation)
{
	playerTimerunInformation.running = false;
	playerTimerunInformation.runName = "";
	playerTimerunInformation.completionTime = -1;
	playerTimerunInformation.previousRecord = 0;
	playerTimerunInformation.startTime = 0;
	playerTimerunInformation.lastRunTimer = cg.time;
}

void ETJump::TimerunView::interrupt()
{
	auto clientNum = atoi(CG_Argv(2));
	interrupt(_playersTimerunInformation[clientNum]);
}

const ETJump::PlayerTimerunInformation* ETJump::TimerunView::currentRun() const
{
	return &_playersTimerunInformation[cg.snap->ps.clientNum];
}

void ETJump::TimerunView::draw()
{
	if (player_drawRunTimer.integer == 0 || !cg.hasTimerun)
	{
		return;
	}

	auto run = currentRun();
	auto startTime = run->startTime;
	auto millis = 0;

	if (run->running)
	{
		millis = cg.time - startTime;
	}
	else
	{
		millis = run->completionTime;
		if (millis == -1)
		{
			millis = 0;
		}
	}

	auto color = &colorWhite;
	
	vec4_t incolor;
	vec4_t ryGreen = { 0.627, 0.941, 0.349, 1.0 };
	vec4_t ryRed = { 0.976, 0.262, 0.262, 1.0 };

	auto range = getTransitionRange(run->previousRecord);
	auto style = ITEM_TEXTSTYLE_NORMAL;
	auto fadeOut = 2000; // 2s fade out
	auto fadeStart = 5000; // 5s pause

	if (etj_runTimerShadow.integer) {
		style = ITEM_TEXTSTYLE_SHADOWED;
	}

	if (run->previousRecord > 0)
	{
		if (millis > run->previousRecord)
		{
			color = &ryRed;
		}
		// add timer color transition when player gets closer to his pb
		else if ( millis + range >= run->previousRecord ) {
			auto start = run->previousRecord - range;
			auto step = (millis - start) / (float)(run->previousRecord - start);
			
			CG_LerpColors(&colorWhite, &ryRed, &incolor, step / 2);
			color = &incolor;
		}
	}

	// set green color for pb time
	if (!run->running && millis && (run->previousRecord > millis || run->previousRecord == -1)) {
		color = &ryGreen;
	}

	auto ms = millis;
	auto minutes = millis / 60000;
	millis -= minutes * 60000;
	auto seconds = millis / 1000;
	millis -= seconds * 1000;

	auto text = (boost::format("%02d:%02d:%03d")
		% minutes
		% seconds
		% millis).str();

	auto textWidth = CG_Text_Width_Ext(text.c_str(), 0.3, 0, &cgs.media.limboFont1) / 2;
	auto x = player_runTimerX.value;
	auto y = player_runTimerY.integer;

	// timer fading/hiding routine
	CG_AdjustPosX(&x);

	if (!run->running && etj_runTimerAutoHide.integer) {
		auto fstart = run->lastRunTimer + fadeStart;
		auto fend = fstart + fadeOut;

		if (fstart < cg.time && fend > cg.time) {

			vec4_t toColor;
			memcpy(&toColor, color, sizeof(toColor));
			toColor[3] = 0;

			auto step = (cg.time - fstart) / (float)(fend - fstart);

			CG_LerpColors(color, &toColor, &incolor, step);
			color = &incolor;
		
		}
		else if(cg.time > fend) {
			// dont draw timer once fading is done
			return;
		}

	}

	if (run->running) {

		if (run->previousRecord != -1 && ms > run->previousRecord)
		{
			pastRecordAnimation(color, text.c_str(), ms, run->previousRecord);
		}
	}

	CG_Text_Paint_Ext(x - textWidth, y, 0.3, 0.3, *color, text.c_str(), 0, 0, style, &cgs.media.limboFont1);
}

int ETJump::TimerunView::getTransitionRange(int previousRunTime)
{
	auto range = 10000;

	if (3 * 1000 > previousRunTime) {
		range = 0;
	} else if (10 * 1000 > previousRunTime) {
		range = 500; // just for a nice short transition effect, could be 0
	}
	else if (30 * 1000 > previousRunTime) {
		range = 2000;
	}
	else if (60 * 1000 > previousRunTime) {
		range = 3500;
	}
	else if (120 * 1000 > previousRunTime) {
		range = 5000;
	}

	return range;
}

void ETJump::TimerunView::pastRecordAnimation(vec4_t *color, const char* text, int timerTime, int record)
{
	auto animationTime = 300;
	
	if (timerTime - record > animationTime) {
		return;
	}

	vec4_t toColor;
	vec4_t incolor;

	auto x = player_runTimerX.value;
	auto y = player_runTimerY.value;

	auto step = ((float)(timerTime - record) / animationTime);
	auto scale = 0.3 + 0.25 * step;

	auto originalTextHeight = CG_Text_Height_Ext(text, 0.3, 0, &cgs.media.limboFont1);
	auto textWidth = CG_Text_Width_Ext(text, scale, 0, &cgs.media.limboFont1) / 2;
	auto textHeight = (CG_Text_Height_Ext(text, scale, 0, &cgs.media.limboFont1) - originalTextHeight) / 2;

	memcpy(&toColor, color, sizeof(toColor));
	toColor[3] = 0;

	CG_LerpColors(color, &toColor, &incolor, step);

	CG_Text_Paint_Ext(x - textWidth, y + textHeight, scale, scale, incolor, text, 0, 0, 0, &cgs.media.limboFont1);
}

bool ETJump::TimerunView::parseServerCommand()
{
	auto argc = trap_Argc();

	if (argc == 1)
	{
		return false;
	}

	char cmd[MAX_TOKEN_CHARS]{};
	trap_Argv(1, cmd, sizeof(cmd));

	if (!Q_stricmp(cmd, "start"))
	{
		start();
	} else if (!Q_stricmp(cmd, "stop"))
	{
		stop();
	}  else if (!Q_stricmp(cmd, "interrupt"))
	{
		interrupt();
	} else
	{
		return false;
	}

	return true;
}