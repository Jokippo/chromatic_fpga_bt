// QSPI_Slave.v

module QSPI_Slave(
    input               QSPI_CLK,
    input               QSPI_CS,
    input               QSPI_MOSI,
    inout               QSPI_MISO,
    input               QSPI_WP,
    input               QSPI_HD,
    input               AUD_MCLK,

    input      [15:0]   LEFT,
    input      [15:0]   RIGHT,
    
    output              qMenuInit,
    output              qDataValid,
    output      [15:0]  qData,
    output  reg [31:0]  qAddress = 'd0,
    output  reg [9:0]   qLength = 'd0,
    output  reg         qCommand = 'd0
);

    // FIFO
    localparam CMD_AUDIO = 10'h015;
    localparam AF_BITS   = 11;                 // 2^11 = 2048 words

    reg [31:0] aud_ram [0:(1<<AF_BITS)-1];
    reg  [AF_BITS-1:0] aud_wr = 0;
    reg  [AF_BITS-1:0] aud_rd = 0;

    wire fifo_empty = (aud_wr == aud_rd);
    wire fifo_full  = (aud_wr + 2'd2 == aud_rd);

    // 44.1 khz clock
    reg [31:0] phase = 32'd0;
    reg AUD_WCLK;

    always @(posedge AUD_MCLK)
    begin
           {AUD_WCLK, phase} <= phase + 32'd22579200;
    end

    always @(posedge AUD_MCLK) begin
        if (pcm_tick && !fifo_full) begin
            aud_ram[aud_wr] <= { LEFT[7:0],LEFT[15:8],RIGHT[7:0],RIGHT[15:8] };
            aud_wr          <= aud_wr + 1'b1;
        end
    end

    // Edge detect on AUD_WCLK
    reg wclk_r;
    wire pcm_tick;

    always @(posedge AUD_MCLK)
    begin
        wclk_r <= AUD_WCLK;
    end
    assign pcm_tick = (wclk_r == 0) & (AUD_WCLK == 1);   // rising edge

    // Audio SPI
    reg miso_oe = 1'b0;
    reg miso_do = 1'b0;
    assign QSPI_MISO = miso_oe ? miso_do : 1'bz;

    reg [15:0] aud_shift = 16'd0;
    reg  [4:0] bit_cnt   = 5'd0;

    reg audio_xfer;

    reg        lr_sel;
    reg [31:0] rd_word;

    // Original
    reg qAddReady = 'd0;
    reg qLenReady = 'd0;

    reg [3:0] qPins_r1 = 'd0;
    reg [7:0] qDataByte = 'd0;
    reg qCyclePhase = 'd0;

    reg qValid = 'd0;
    wire [3:0] qPins = {
                    QSPI_HD,
                    QSPI_WP,
                    QSPI_MISO,
                    QSPI_MOSI};

    reg [7:0] qCycleCount = 'd0;

    reg qMenuInit1 = 1'd0;
    reg qMenuInit2 = 1'd0;
    assign qMenuInit = qMenuInit2;

    always@(posedge QSPI_CLK or posedge QSPI_CS)
    begin
        if(QSPI_CS)
        begin
            qCyclePhase <= 1'd0;
            qValid      <= 1'd0;
            qCycleCount <= 1'd0;
            qCommand    <= 1'd0;
            qAddReady   <= 1'd0;
            qLenReady   <= 1'd0;
            miso_oe     <= 1'b0;
            audio_xfer  <= 1'b0;
        end
        else
        begin
            if(qCycleCount <= 50)
                qCycleCount   <= qCycleCount + 1'd1;
            
            // Command
            if(qCycleCount == 0)
                qCommand <= QSPI_MOSI;

            if((qCycleCount >= 1) && (qCycleCount <= 10))
                qLength <= {qLength[8:0], QSPI_MOSI};
                
            if((qCycleCount >= 11) && (qCycleCount <= 42))
                qAddress <= {qAddress[30:0], QSPI_MOSI};
                
            if(qCycleCount == 11)
                qLenReady <= 1'd1;
            else
                qLenReady <= 1'd0;

            if(qCycleCount == 43)
            begin
                qAddReady <= 1'd1;
                if(qAddress == 0) // first row
                begin
                    qMenuInit1  <=  1'd1;
                    if(qMenuInit1)
                        qMenuInit2 <= 1'd1;
                end
            end
            else
                qAddReady <= 1'd0;
                
            // QSPI data
            if(qCycleCount >= 46)
            begin
                qCyclePhase      <= ~qCyclePhase;
                if(qCyclePhase)
                begin
                    qDataByte   <= {qPins_r1,qPins};
                    qValid      <= 1'd1;
                end
                else
                begin
                    qPins_r1    <= qPins;
                    qValid      <= 1'd0;
                end
            end

            if (qLenReady && qCommand==1'b0 && qLength==CMD_AUDIO)
            audio_xfer <= 1'b1;

            // Audio read transfer
            if (audio_xfer) begin
                if (qCycleCount == 16) begin
                    if (!fifo_empty) begin
                        rd_word  <= aud_ram[aud_rd];
                        aud_shift <= aud_ram[aud_rd][31:16];    // Left
                        lr_sel   <= 1'b1;
                    end else begin
                        aud_shift <= 16'h0000;
                    end
                    bit_cnt <= 5'd15;
                    miso_oe <= 1'b1;

                end else if (qCycleCount >= 17) begin
                    miso_do   <= aud_shift[15];
                    aud_shift <= {aud_shift[14:0], 1'b0};

                    if (bit_cnt == 0) begin
                        if (!fifo_empty) begin
                            if (lr_sel) begin
                                aud_shift <= rd_word[15:0];     // Right
                                aud_rd    <= aud_rd + 1'b1;  
                                lr_sel    <= 1'b0;
                            end else begin
                                rd_word   <= aud_ram[aud_rd];
                                aud_shift <= aud_ram[aud_rd][31:16];
                                lr_sel    <= 1'b1;
                            end
                        end else begin
                            aud_shift <= 16'h0000;
                        end
                        bit_cnt <= 5'd15;
                    end else begin
                        bit_cnt <= bit_cnt - 1'b1;
                    end
                end
            end
        end
    end

    reg [7:0] qDataByte_r1 = 'd0;
    reg QSPI_VALID_phase = 'd0;
    always@(posedge QSPI_CLK or posedge QSPI_CS)
        if(QSPI_CS)
            QSPI_VALID_phase <= 'd0;
        else
            if(qValid)
            begin
                qDataByte_r1 <= qDataByte;
                QSPI_VALID_phase <= ~QSPI_VALID_phase;
            end

    assign qDataValid = qValid&QSPI_VALID_phase;
    assign qData = {qDataByte,qDataByte_r1};

endmodule