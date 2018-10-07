var gui = {}
gui.messages = null;

//se preia limba sistemului si se intializeaza interfata
$(document).ready(function(){
  get_lang('messages', init_interface)
});

function init_interface(lang_file){
  
  //se preia denumirea pentru butonul start
  console.log(lang_file)
    $.ajax({
      url: lang_file,
      dataType: 'json', 
      success: function(data){
        $('div>h1').html(data.messages.main);
        gui.messages = data.messages;
      },
      error: function(){
        console.log('error loading file');
      },
      async: false
    });
  $('#start_quest').html(gui.messages.start)
  /*Comenzile pentru afisarea hartii
  $('#show_map').html(gui.messages.showmap)

  $('#show_map').click(function(){
    web_cmd_callback("SHOW_MAP");
    setTimeout(function(){ web_cmd_callback("SHOW_LOGO"); }, 3000);
  });
  */

  $('.btn-index-game').click(function(){
    // // load questionnaire page
    //window.location = 'questions.html';
    //cand se apasa pe butonul de start intram in zona de chestionar
    //asa ca acrtivam chestionarul si initializam intrebarile
    document.getElementById('start_page').hidden = true;
    document.getElementById('questions_page').hidden = false;
    init_game(lang_file);
  });
};

