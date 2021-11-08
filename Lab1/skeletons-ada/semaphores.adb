-- Package: Semaphores
--
-- ==> Complete the code at the indicated places

package body Semaphores is
   protected body CountingSemaphore is
      entry Wait -- To be completed
         when Count > 0 is -- Guard
      begin 
         Count := Count - 1;
      end Wait;


      entry Signal -- To be completed
         when Count < MaxCount is -- Guard
      begin
         Count := Count + 1;
      end Signal;

   end CountingSemaphore;
end Semaphores;

