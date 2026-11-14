# genx320_setup
Setup code for Genx320 event camera  

Events are polled super quickly (possibly within microsecond range), but due to not being RTOS (micropython), probably not very consistently. Once a stream of events is captured (up to 4096 events as capped in the initialization), the loop binary searches through the event stream to find the first index that has (or is closest to) the 20ms cutoff (specified as win_end). It then stores those event streams within the initial captured event up to the cutoff timestamp event, which is either the entire event stream if the largest timestamp isn't 20ms or the timestamp closest to 20ms if it is, leaving the remaining events to be discarded as overflow. Once the cutoff is reached, the window is closed and the win_idx increments, repeating the cycle until the max N_WINDOWS is iterated through. 

11/14:
- Re-resetting the camera mode of the genx320 RIGHT BEFORE event capture is done WITH no delay (so no 500ms to settle) signifcantly improves 0-event captures; for 250-windows samples, # of 0-event frames decrease from 40-80 to less than 10
- Anti flicker makes no difference for this re-reset method
- No existing ERC API (yet). Could be due to how the code is written (see above).
- Bias makes no difference other than (sometimes) increasing 0-event issues
- Digital event mask TBD
- Some things to consider later:
  - Figure out digital event mask
  - Figure out ERC
  - Figure out getting usable images for labeling (not just live view)