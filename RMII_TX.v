// RMII PHY-TX Interface

module RMII_TX #(
        parameter IFG = 48 // Interframe Gap
    )(
        // Asynchronous Reset
        input  wire        arst_n,

        // FIFO signal
        input  wire        fifo_aempty,
        input  wire [7:0]  fifo_dout,
        input  wire        fifo_empty,
        output wire        fifo_rden,
        // Original FIFO signal
        input  wire        fifo_EOD_out,

        // RMII signal
        input  wire        REF_CLK,
        output wire        TXD0,
        output wire        TXD1,
        output wire        TX_EN,
        input  wire        CRS_DV,

        // monitor signal
        output wire [15:0] succ_tx_count_gray, // success TX frame counter
        output wire [15:0] fail_tx_count_gray, // fail TX frame counter
    );

    reg TXD0_reg;
    reg TXD1_reg;
    reg TXEN_reg;

    assign TXD0  = TXD0_reg;
    assign TXD1  = TXD1_reg;
    assign TX_EN = TXEN_reg;

    reg [15:0] succ_tx_count_reg; // raw binary counter
    reg [15:0] fail_tx_count_reg;
    my_bin2gray #(
                    .WIDTH(16)
                )
                succ_gray(
                    .din(succ_tx_count_reg),
                    .dout(succ_tx_count_gray)
                );
    my_bin2gray #(
                    .WIDTH(16)
                )
                fail_gray(
                    .din(fail_tx_count_reg),
                    .dout(fail_tx_count_gray)
                );

    reg [1:0] STATE_reg;
    localparam S_IDLE     = 2'd0,
               S_PREAMBLE = 2'd1,
               S_BODY     = 2'd2,
               S_END      = 2'd3;

    // general purpose counter
    reg [7:0] cnt_reg;

    reg [1:0] STATE_next;
    reg TXD0_next, TXD1_next, TXEN_next;
    reg [15:0] succ_tx_count_next, fail_tx_count_next;
    reg [7:0] cnt_next;

    reg [7:0] latched_fifo_dout_reg;
    reg [7:0] latched_fifo_dout_next;
    reg latched_fifo_EOD_reg;
    reg latched_fifo_EOD_next;
    reg latched_fifo_empty_reg;
    reg latched_fifo_empty_next;

    always @* begin
        // default value
        STATE_next = STATE_reg;
        TXD0_next  = 1'b0;
        TXD1_next  = 1'b0;
        TXEN_next  = 1'b0;
        succ_tx_count_next = succ_tx_count_reg;
        fail_tx_count_next = fail_tx_count_reg;
        cnt_next = cnt_reg;
        fifo_rden = 1'b0;
        latched_fifo_dout_next  = latched_fifo_dout_reg;
        latched_fifo_EOD_next   = latched_fifo_EOD_reg;
        latched_fifo_empty_next = latched_fifo_empty_reg;

        case (STATE_reg)
            S_IDLE:
            begin
                TXD0_next = 1'b0;
                TXD1_next = 1'b0;
                TXEN_next = 1'b0;
                if (~fifo_aempty) begin // FIFOにある程度データが溜まると送出を開始
                    STATE_next = S_PREAMBLE;
                end
            end

            S_PREAMBLE:
            begin
                cnt_next = cnt_reg + 1'b1;
                TXEN_next = 1'b1;
                if (cnt_reg < 8'd31) begin
                    TXD0_next = 1'b1;
                    TXD1_next = 1'b0;
                end
                else if (cnt_reg == 8'd31) begin
                    TXD0_next = 1'b1;
                    TXD1_next = 1'b1;
                    cnt_next = 8'b0;
                    STATE_next = S_BODY;
                    // fetch new byte (needed in next state)
                    latched_fifo_dout_next  = fifo_dout;
                    latched_fifo_EOD_next   = fifo_EOD_out;
                    latched_fifo_empty_next = fifo_empty;
                end
            end

            S_BODY: // フレーム本体を送出
            begin
                cnt_next = cnt_reg + 1'b1;
                TXEN_next = 1'b1;
                case (cnt_reg[1:0])
                    2'd0:
                    begin
                        TXD0_next = latched_fifo_dout_reg[0];
                        TXD1_next = latched_fifo_dout_reg[1];
                        if (latched_fifo_empty_reg & ~latched_fifo_EOD_reg) begin // // EODが来る前にFIFOが空になると送出中止
                            fail_tx_count_next = fail_tx_count_next + 1'b1;
                            STATE_next = S_END;
                        end
                    end

                    2'd1:
                    begin
                        TXD0_next = latched_fifo_dout_reg[2];
                        TXD1_next = latched_fifo_dout_reg[3];
                    end

                    2'd2: // fifo_rdenをassert
                    begin
                        TXD0_next = latched_fifo_dout_reg[4];
                        TXD1_next = latched_fifo_dout_reg[5];
                        fifo_rden = 1'b1;
                    end

                    2'd3: // FIFOの出力をラッチ
                    begin
                        TXD0_next = latched_fifo_dout_reg[6];
                        TXD1_next = latched_fifo_dout_reg[7];
                        cnt_next = 8'b0;
                        latched_fifo_dout_next = fifo_dout;
                        latched_fifo_EOD_next = fifo_EOD_out;
                        latched_fifo_empty_next = fifo_empty;
                        if (latched_fifo_EOD_reg) begin // 先ほどのデータがframe終端なら送出終了
                            succ_tx_count_next = succ_tx_count_next + 1'b1;
                            STATE_next = S_END;
                        end
                    end

                    default:
                    begin
                        TXD0_next = 1'b0;
                        TXD1_next = 1'b0;
                        TXEN_next = 1'b0;
                    end
                endcase
            end

            S_END: // IFGの間待機してからアイドル状態に遷移
            begin
                TXD0_next = 1'b0;
                TXD1_next = 1'b0;
                TXEN_next = 1'b0;
                cnt_next = cnt_reg + 1'b1;
                if (cnt_reg == (IFG - 2)) begin
                    cnt_next = 8'b0;
                    STATE_next = S_IDLE;
                end
            end

            default: // undefined state => go to S_END
            begin
                STATE_next = S_END;
            end
        endcase
    end

    always @(posedge REF_CLK or negedge arst_n) begin
        if (~arst_n) begin
            STATE_reg <= S_IDLE;
            TXD0_reg  <= 1'b0;
            TXD1_reg  <= 1'b0;
            TXEN_reg  <= 1'b0;
            succ_tx_count_reg <= 16'b0;
            fail_tx_count_reg <= 16'b0;
            cnt_reg <= 8'b0;
            latched_fifo_dout_reg  <= 8'b0;
            latched_fifo_EOD_reg   <= 1'b0;
            latched_fifo_empty_reg <= 1'b0;
        end
        else begin
            STATE_reg <= STATE_next;
            TXD0_reg  <= TXD0_next;
            TXD1_reg  <= TXD1_next;
            TXEN_reg  <= TXEN_next;
            succ_tx_count_reg <= succ_tx_count_next;
            fail_tx_count_reg <= fail_tx_count_next;
            cnt_reg <= cnt_next;
            latched_fifo_dout_reg  <= latched_fifo_dout_next;
            latched_fifo_EOD_reg   <= latched_fifo_EOD_next;
            latched_fifo_empty_reg <= latched_fifo_empty_next;
        end
    end
endmodule // RMII_TX
