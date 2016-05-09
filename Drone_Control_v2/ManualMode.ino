void ManualMode()
{
  if (thrSpeed > thro_in)
    thrSpeed = thro_in;
  SendCommands(thro_in, eleo_in, aleo_in, rudd_in);

  DebugManualMode();
}

